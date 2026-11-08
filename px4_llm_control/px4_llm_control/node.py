"""One ROS executor owns flight state; a worker performs only HTTP inference."""
import json
import math
import os
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleCommandAck, VehicleLocalPosition, VehicleStatus,
                          VehicleLandDetected)
from .planner import request_plan, target_for


class LLMControl(Node):
    def __init__(self):
        super().__init__('px4_llm_control')
        for name, value in {'model': 'deepseek-chat', 'base_url': 'https://api.deepseek.com',
                            'api_timeout': 30.0, 'dry_run': False}.items():
            self.declare_parameter(name, value)
        self.key = os.environ.get('DEEPSEEK_API_KEY', '')
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.generation = 0
        self.pending_generation = 0
        self.position = self.status = self.land = None
        self.received = {}
        self.origin = self.target = None
        self.yaw = 0.0
        self.resets = None
        self.phase = 'idle'
        self.actions = []
        self.step = None
        self.started = time.monotonic()
        self.last_command = 0.0
        self.reached = None
        self.owned = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command_v1', qos)
        for cls, topic, name in [
            (VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', 'position'),
            (VehicleStatus, '/fmu/out/vehicle_status_v5', 'status'),
            (VehicleLandDetected, '/fmu/out/vehicle_land_detected', 'land')]:
            self.create_subscription(cls, topic, lambda msg, n=name: self.telemetry(n, msg), qos)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack_v2', self.ack, qos)
        self.events = self.create_publisher(String, '/px4_llm/status', 10)
        self.plans = self.create_publisher(String, '/px4_llm/plan', 10)
        self.create_subscription(String, '/px4_llm/command', self.instruction, 10)
        self.create_subscription(String, '/px4_llm/emergency', self.emergency, 10)
        self.create_timer(0.05, self.tick)
        self.event('ready', '订阅 /px4_llm/command；仅用于 SITL。请关闭其他 Offboard 控制器。')

    def event(self, event, detail):
        self.events.publish(String(data=json.dumps(
            {'event': event, 'phase': self.phase, 'detail': detail}, ensure_ascii=False)))
        self.get_logger().info(f'{event}: {detail}')

    def telemetry(self, name, msg):
        setattr(self, name, msg)
        self.received[name] = time.monotonic()

    def fresh(self, name):
        return time.monotonic() - self.received.get(name, -math.inf) < 2.0

    def valid(self):
        return (all(self.fresh(n) for n in ('position', 'status', 'land'))
                and self.position.xy_valid and self.position.z_valid
                and all(math.isfinite(v) for v in
                        (self.position.x, self.position.y, self.position.z, self.position.heading)))

    def xyz(self):
        return [self.position.x, self.position.y, self.position.z]

    def armed(self):
        return self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def reset_counts(self):
        return (self.position.xy_reset_counter, self.position.z_reset_counter,
                self.position.heading_reset_counter)

    def instruction(self, msg):
        text = msg.data.strip()
        if not text or len(text) > 2000:
            self.event('rejected', '指令长度必须为1到2000字符'); return
        if self.future is not None or self.actions or self.step or self.phase not in ('idle', 'hold'):
            self.event('busy', '已有推理或动作执行；需要打断请使用 emergency'); return
        if not self.key:
            self.event('rejected', '请先设置 DEEPSEEK_API_KEY，再重启节点'); return
        if not self.valid() or self.status.failsafe:
            self.event('rejected', '等待有效、新鲜的位置、状态和着陆数据'); return
        if not self.owned and (self.armed() or not self.land.landed):
            self.event('rejected', '请在地面未解锁时启动本节点，不接管其他控制器'); return
        self.pending_generation = self.generation
        self.future = self.pool.submit(request_plan, text,
            {'position_ned': self.xyz(), 'heading_deg': math.degrees(self.position.heading),
             'airborne': bool(self.owned), 'phase': self.phase}, self.key,
            self.get_parameter('model').value, self.get_parameter('base_url').value,
            float(self.get_parameter('api_timeout').value))
        self.event('planning', text)

    def emergency(self, msg):
        if msg.data.strip().lower() not in ('land', 'stop'):
            self.event('rejected', 'emergency 支持 land 或 stop'); return
        self.generation += 1  # Discard any late network result.
        self.actions = []
        self.step = None
        if msg.data.strip().lower() == 'land':
            self.begin_land('用户要求降落')
        elif self.owned and self.valid() and self.phase != 'landing':
            self.target, self.yaw = self.xyz(), self.position.heading
            self.phase = 'hold'
            self.event('stopped', '已清空任务，保持当前位置')
        elif self.phase in ('prestream', 'mode', 'arm'):
            self.begin_land('取消起飞过程')
        else:
            self.event('stopped', '已取消任务；当前阶段不能切换为悬停')

    def command(self, command, p1=0.0, p2=0.0):
        if self.get_parameter('dry_run').value:
            self.event('dry_run', f'跳过 VehicleCommand {command}')
            return
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command, msg.param1, msg.param2 = command, float(p1), float(p2)
        msg.target_system = msg.target_component = 1
        msg.source_system = msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
        self.last_command = time.monotonic()

    def ack(self, msg):
        if msg.command in (176, 400, 21):
            self.event('ack', {'command': msg.command, 'result': msg.result})

    def begin_land(self, reason):
        self.generation += 1
        self.actions, self.step = [], None
        self.phase, self.started, self.last_command = 'landing', time.monotonic(), 0.0
        self.event('landing', reason)

    def accept(self, plan):
        actions = plan['actions']
        self.plans.publish(String(data=json.dumps(plan, ensure_ascii=False)))
        if not actions:
            self.event('rejected', plan['explanation']); return
        if not self.valid() or self.status.failsafe:
            raise ValueError('推理后遥测无效')
        if self.phase not in ('idle', 'hold'):
            raise ValueError('推理期间控制状态已改变')
        airborne = self.owned
        if not airborne and (self.armed() or not self.land.landed):
            raise ValueError('推理期间飞机已离开地面或被其他节点解锁')
        if airborne and actions[0]['action'] == 'takeoff':
            raise ValueError('空中不能重复起飞')
        if not airborne and actions[0]['action'] not in ('takeoff', 'land'):
            raise ValueError('地面任务需要先起飞')
        origin = self.origin if airborne else self.xyz()
        pos, yaw = self.xyz(), self.position.heading
        # Validate the entire route before sending any flight command.
        for step in actions:
            if step['action'] != 'land':
                pos, yaw = target_for(step, pos, yaw, origin)
        if self.get_parameter('dry_run').value:
            self.event('dry_run', '计划校验通过，未发送控制命令'); return
        self.origin = origin
        self.actions = list(actions)
        if not airborne and actions[0]['action'] == 'takeoff':
            self.target = self.xyz()
            self.yaw = self.position.heading
            self.resets = self.reset_counts()
            self.phase, self.started = 'prestream', time.monotonic()
        else:
            self.next_step()

    def next_step(self):
        if not self.actions:
            self.step, self.phase = None, 'hold'
            self.event('completed', '任务完成，持续悬停，等待下一条指令'); return
        self.step = self.actions.pop(0)
        if self.step['action'] == 'land':
            self.begin_land('执行计划降落'); return
        self.target, self.yaw = target_for(self.step, self.xyz(), self.position.heading, self.origin)
        self.phase, self.started, self.reached = 'executing', time.monotonic(), None
        self.event('executing', self.step)

    def stream(self):
        stamp = self.get_clock().now().nanoseconds // 1000
        mode = OffboardControlMode(timestamp=stamp, position=True)
        self.mode_pub.publish(mode)
        msg = TrajectorySetpoint()
        msg.timestamp, msg.position = stamp, [float(v) for v in self.target]
        msg.velocity = msg.acceleration = msg.jerk = [math.nan] * 3
        msg.yaw, msg.yawspeed = float(self.yaw), math.nan
        self.setpoint_pub.publish(msg)

    def tick(self):
        now = time.monotonic()
        if self.future is not None and self.future.done():
            future, self.future = self.future, None
            if self.pending_generation == self.generation:
                try:
                    self.accept(future.result())
                except Exception as exc:
                    self.event('error', f'计划未执行: {type(exc).__name__}: {exc}')
        if self.phase == 'landing':
            if self.fresh('status') and self.fresh('land') and self.land.landed and not self.armed():
                self.phase, self.owned = 'idle', False
                self.origin = self.target = None
                self.event('landed', '已着陆并上锁'); return
            if now - self.started > 90:
                self.phase, self.owned = 'idle', False
                self.event('error', '降落超时，请通过 QGC 检查并接管'); return
            if now - self.last_command > 1 and (not self.fresh('status') or self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND):
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            return
        if self.phase == 'idle':
            return
        if not self.valid() or self.status.failsafe or self.reset_counts() != self.resets:
            self.begin_land('遥测过期/位置重置/失效保护，取消任务并请求降落'); return
        if self.owned and (not self.armed() or self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.generation += 1
            self.actions, self.step = [], None
            self.phase, self.owned = 'idle', False
            self.event('released', '已离开 Offboard 或上锁，停止输出；通过 QGC 接管'); return
        self.stream()
        if self.phase in ('prestream', 'mode', 'arm') and now - self.started > 15:
            self.begin_land('Offboard/解锁超时，请检查 ACK 和 QGC'); return
        if self.phase == 'prestream' and now - self.started >= 2:
            self.phase = 'mode'
            self.last_command = 0.0
        elif self.phase == 'mode':
            if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.phase, self.last_command = 'arm', 0.0
            elif now - self.last_command > 1:
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
        elif self.phase == 'arm':
            if self.armed():
                self.owned = True
                try:
                    self.next_step()
                except ValueError as exc:
                    self.begin_land(str(exc))
            elif now - self.last_command > 1:
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
        elif self.phase == 'executing':
            if now - self.started > 180:
                self.begin_land('动作执行超时'); return
            error = math.dist(self.xyz(), self.target)
            yaw_error = abs(math.atan2(math.sin(self.position.heading - self.yaw),
                                       math.cos(self.position.heading - self.yaw)))
            speed = math.sqrt(sum(v*v for v in (self.position.vx, self.position.vy, self.position.vz)))
            arrived = error < 0.25 and yaw_error < math.radians(8) and speed < 0.3
            if arrived:
                if self.reached is None:
                    self.reached = now
                duration = self.step.get('seconds', 0.5)
                if now - self.reached >= duration:
                    try:
                        self.next_step()
                    except ValueError as exc:
                        self.begin_land(str(exc))
            else:
                self.reached = None


def main(args=None):
    rclpy.init(args=args)
    node = LLMControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.owned:
            node.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            node.get_logger().warning('退出时已请求降落，请在 QGC 确认')
    finally:
        node.pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
