"""One ROS executor owns flight state; a worker performs only HTTP inference."""
import json
import math
import os
import time
from concurrent.futures import ThreadPoolExecutor
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleCommandAck, VehicleLocalPosition, VehicleStatus,
                          VehicleLandDetected, VehicleAttitude)
from .planner import request_plan, target_for, validate_plan, SHAPES
from .trajectory import Circuit, Line, Waypoints, bearing, wrap


class LLMControl(Node):
    def __init__(self):
        super().__init__('px4_llm_control')
        for name, value in {'model': 'deepseek-chat', 'base_url': 'https://api.deepseek.com',
                            'api_timeout': 60.0, 'dry_run': False, 'max_tokens': 8192,
                            'max_acceleration': 2.0, 'max_yaw_rate_deg': 90.0}.items():
            self.declare_parameter(name, value)
        self.acceleration = float(self.get_parameter('max_acceleration').value)
        self.yaw_rate = math.radians(float(self.get_parameter('max_yaw_rate_deg').value))
        if not 0.1 <= self.acceleration <= 4 or not 5 <= math.degrees(self.yaw_rate) <= 180:
            raise ValueError('max_acceleration范围0.1到4，max_yaw_rate_deg范围5到180')
        self.motion = None
        self.velocity, self.acceleration_ff = [0.0]*3, [0.0]*3
        self.yawspeed = 0.0
        self.motion_time = 0.0
        self.last_tick = time.monotonic()
        self.attitude = None
        self.pitch_request = None
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
        self.actions = deque()
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
            (VehicleLandDetected, '/fmu/out/vehicle_land_detected', 'land'),
            (VehicleAttitude, '/fmu/out/vehicle_attitude', 'attitude')]:
            self.create_subscription(cls, topic, lambda msg, n=name: self.telemetry(n, msg), qos)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack_v2', self.ack, qos)
        self.events = self.create_publisher(String, '/px4_llm/status', 10)
        self.plans = self.create_publisher(String, '/px4_llm/plan', 10)
        self.create_subscription(String, '/px4_llm/command', self.instruction, 10)
        self.create_subscription(String, '/px4_llm/emergency', self.emergency, 10)
        self.feedback_pub = self.create_publisher(String, '/px4_llm/feedback', 10)
        self.create_timer(0.5, self.feedback)
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
        if not text:
            self.event('rejected', '指令不能为空'); return
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
             'airborne': bool(self.owned), 'phase': self.phase,
             'takeoff_origin_ned': self.origin,
             'max_acceleration_m_s2': self.acceleration}, self.key,
            self.get_parameter('model').value, self.get_parameter('base_url').value,
            float(self.get_parameter('api_timeout').value),
            int(self.get_parameter('max_tokens').value))
        self.event('planning', text)

    def emergency(self, msg):
        if msg.data.strip().lower() not in ('land', 'stop'):
            self.event('rejected', 'emergency 支持 land 或 stop'); return
        self.generation += 1  # Discard any late network result.
        self.actions = deque()
        self.step = None
        self.clear_motion()
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
        self.actions, self.step = deque(), None
        self.clear_motion()
        self.phase, self.started, self.last_command = 'landing', time.monotonic(), 0.0
        self.event('landing', reason)

    def accept(self, plan):
        actions = validate_plan(plan)
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
                motion = self.make_motion(step, pos, yaw, origin)
                if motion is not None:
                    pos = list(motion.end)
                    if getattr(motion, 'heading', 'hold') == 'center':
                        yaw = bearing(pos, motion.center, yaw)
                    elif getattr(motion, 'heading', 'hold') == 'tangent':
                        _, velocity, _ = motion.sample(max(0.0, motion.duration-0.001))
                        if math.hypot(*velocity[:2]) > 1e-8:
                            yaw = math.atan2(velocity[1], velocity[0])
                else:
                    pos, yaw = target_for(step, pos, yaw, origin)
        self.pitch_request = plan.get('attitude', {}).get('pitch_deg')
        if self.pitch_request is not None:
            self.event('constraint_warning', '俯仰0度为参考意图，位置控制不能强制；加减速/转弯会倾斜，实际值见feedback')
        if any(step['action'] in SHAPES for step in actions):
            self.event('trajectory_notice', '速度为参考巡航目标；多边形顶点减速到零，圆形受向心加速度限速')
        if self.get_parameter('dry_run').value:
            self.event('dry_run', '计划校验通过，未发送控制命令'); return
        self.origin = origin
        self.actions = deque(actions)
        if not airborne and actions[0]['action'] == 'takeoff':
            self.target = self.xyz()
            self.yaw = self.position.heading
            self.resets = self.reset_counts()
            self.phase, self.started = 'prestream', time.monotonic()
        else:
            self.next_step()

    def next_step(self):
        if not self.actions:
            self.clear_motion()
            self.step, self.phase = None, 'hold'
            self.event('completed', '任务完成，持续悬停，等待下一条指令'); return
        self.clear_motion()
        self.step = self.actions.popleft()
        if self.step['action'] == 'land':
            self.begin_land('执行计划降落'); return
        self.motion = self.make_motion(self.step, self.xyz(), self.position.heading, self.origin)
        if self.motion is not None:
            self.target, self.velocity, self.acceleration_ff = self.motion.sample(0)
            self.yaw = self.position.heading
        else:
            self.target, self.yaw = target_for(self.step, self.xyz(), self.position.heading, self.origin)
        self.phase, self.started, self.reached = 'executing', time.monotonic(), None
        self.event('executing', self.step)
        if self.motion is not None:
            self.event('trajectory', {'duration_s': self.motion.duration,
                                     'cruise_m_s': getattr(self.motion, 'speed', self.step.get('speed', 1.0))})

    def clear_motion(self):
        self.motion = None
        self.motion_time = 0.0
        self.velocity, self.acceleration_ff = [0.0]*3, [0.0]*3
        self.yawspeed = 0.0

    def make_motion(self, step, position, yaw, origin):
        if step['action'] in SHAPES:
            return Circuit(step, position, origin, self.acceleration)
        if step['action'] == 'path':
            return Waypoints(step, position, origin, self.acceleration)
        if step['action'] in ('takeoff', 'move'):
            end, _ = target_for(step, position, yaw, origin)
            # Conservative vertical rate for takeoff/relative climbing segments.
            speed = step.get('speed', 1.0)
            if abs(end[2]-position[2]) > 0.01:
                speed = min(speed, 2.0)
            return Line(position, end, speed, self.acceleration)
        return None

    def feedback(self):
        if not self.fresh('position'):
            return
        pitch = None
        if self.fresh('attitude'):
            w, x, y, z = self.attitude.q
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))))
        detail = {'phase': self.phase, 'position_ned': self.xyz(),
                  'reference_ned': self.target, 'velocity_reference': self.velocity,
                  'actual_speed_m_s': math.hypot(self.position.vx, self.position.vy, self.position.vz),
                  'yaw_reference_deg': math.degrees(self.yaw),
                  'yaw_actual_deg': math.degrees(self.position.heading),
                  'pitch_actual_deg': pitch, 'pitch_request_deg': self.pitch_request,
                  'pitch_enforced': False, 'motion_time_s': self.motion_time,
                  'remaining_actions': len(self.actions)}
        self.feedback_pub.publish(String(data=json.dumps(detail, ensure_ascii=False)))

    def stream(self):
        stamp = self.get_clock().now().nanoseconds // 1000
        mode = OffboardControlMode(timestamp=stamp, position=True)
        self.mode_pub.publish(mode)
        msg = TrajectorySetpoint()
        msg.timestamp, msg.position = stamp, [float(v) for v in self.target]
        msg.velocity = [float(v) for v in self.velocity]
        msg.acceleration = [float(v) for v in self.acceleration_ff]
        msg.jerk = [math.nan] * 3
        msg.yaw, msg.yawspeed = float(self.yaw), float(self.yawspeed)
        self.setpoint_pub.publish(msg)

    def tick(self):
        now = time.monotonic()
        dt = min(max(now-self.last_tick, 0.0), 0.1)
        self.last_tick = now
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
            self.actions, self.step = deque(), None
            self.phase, self.owned = 'idle', False
            self.event('released', '已离开 Offboard 或上锁，停止输出；通过 QGC 接管'); return
        if self.phase == 'executing' and self.motion is not None:
            tracking_error = math.dist(self.xyz(), self.target)
            if tracking_error > 4:
                self.begin_land('轨迹跟踪误差超过4米'); return
            # Pause reference progress if PX4 lags; continue the heartbeat and hold reference.
            paused = tracking_error > 1.5 or abs(wrap(self.position.heading-self.yaw)) > math.radians(40)
            if not paused:
                self.motion_time += dt
            self.target, self.velocity, self.acceleration_ff = self.motion.sample(self.motion_time)
            if paused:
                self.velocity, self.acceleration_ff = [0.0]*3, [0.0]*3
            desired_yaw = self.yaw
            heading = getattr(self.motion, 'heading', 'hold')
            if heading == 'center':
                desired_yaw = bearing(self.xyz(), self.motion.center, self.yaw)
            elif heading == 'tangent' and math.hypot(*self.velocity[:2]) > 0.05:
                desired_yaw = math.atan2(self.velocity[1], self.velocity[0])
            change = max(-self.yaw_rate*dt, min(self.yaw_rate*dt, wrap(desired_yaw-self.yaw)))
            self.yaw = wrap(self.yaw+change)
            self.yawspeed = change/dt if dt else 0.0
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
            budget = (self.motion.duration if self.motion is not None else self.step.get('seconds', 0)) + 120
            if now - self.started > budget:
                self.begin_land('动作执行超时'); return
            error = math.dist(self.xyz(), self.target)
            arrival_yaw = self.yaw
            if self.motion is not None and getattr(self.motion, 'heading', 'hold') == 'center':
                arrival_yaw = bearing(self.xyz(), self.motion.center, self.yaw)
            yaw_error = abs(wrap(self.position.heading-arrival_yaw))
            speed = math.sqrt(sum(v*v for v in (self.position.vx, self.position.vy, self.position.vz)))
            trajectory_done = self.motion is None or self.motion_time >= self.motion.duration
            arrived = trajectory_done and error < 0.25 and yaw_error < math.radians(8) and speed < 0.3
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
