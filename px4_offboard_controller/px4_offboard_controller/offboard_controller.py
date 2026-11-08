#!/usr/bin/env python3
"""PX4 Offboard: takeoff, hover, land.

默认流程：
1. 等待 PX4 局部位置有效；
2. 连续发送位置设定点和 OffboardControlMode；
3. 请求 Offboard；
4. 解锁；
5. 飞到指定相对高度并悬停 10 秒；
6. 发送降落命令。

PX4 使用 NED 坐标系：目标高度 2 m 对应 z = -2.0。
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleLandDetected
)


class FlightStage(Enum):
    WAIT_FOR_ESTIMATOR = auto()
    STREAM_SETPOINTS = auto()
    REQUEST_OFFBOARD = auto()
    REQUEST_ARM = auto()
    TAKEOFF = auto()
    HOVER = auto()
    LAND = auto()
    REQUEST_DISARM = auto()
    FINISHED = auto()
    ERROR = auto()


class PX4OffboardController(Node):
    def __init__(self):
        super().__init__('px4_offboard_controller')

        self.declare_parameter('target_height',    2.0)    # 设定高度
        self.declare_parameter('yaw_deg',          0.0)    # 设定偏航角
        self.declare_parameter('hover_seconds',    10.0)   # 设定悬停时间
        self.declare_parameter('setpoint_rate_hz', 20.0)   # 设定预发送频率 (初始化PX4后，需要已一个频率，如20Hz，持续发送 初始设定点)
                                                           # 以20Hz发布：/fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint


        self.target_height = float(self.get_parameter('target_height').value)     
        self.yaw_rad = math.radians(float(self.get_parameter('yaw_deg').value))
        self.hover_seconds = float(self.get_parameter('hover_seconds').value)
        rate_hz = float(self.get_parameter('setpoint_rate_hz').value)

        if self.target_height <= 0.2:
            raise ValueError('target_height 必须大于 0.2 米')
        if self.hover_seconds < 0.0:
            raise ValueError('hover_seconds 不能为负数')
        if not 5.0 <= rate_hz <= 50.0:
            raise ValueError('setpoint_rate_hz 建议设置在 5 到 50 Hz 之间')

        # PX4 输出通常使用 Best Effort QoS。
        px4_out_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        # PX4 输入话题使用可靠、易获取的发布 QoS。
        px4_in_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # PX4输入话题： /fmu/in/
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_in_qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_in_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command_v1', px4_in_qos)  # 这里写错了，后面应该有'_v1'

        # PX4输出话题： /fmu/out/
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_out_qos,
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v5',
            self.status_callback,
            px4_out_qos,
        )
        self.ack_sub = self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack_v2',
            self.ack_callback,
            px4_out_qos,
        )

        self.land_sub = self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self.land_callback,
            px4_out_qos
        )

        self.position = None
        self.status = None
        self.land_detected = None
        self.last_ack = {}
        self.stage = FlightStage.WAIT_FOR_ESTIMATOR
        self.stage_started_at = self.get_clock().now()  # 记录时间
        self.hover_started_at = None
        self.land_sent = False
        self.offboard_request_sent = False
        self.arm_request_sent = False
        self.disarm_request_sent = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_z = 0.0
        self.log_counter = 0

        self.timer = self.create_timer(1.0 / rate_hz, self.control_loop)
        self.get_logger().info(
            f'Controller started: height={self.target_height:.2f} m, '
            f'yaw={math.degrees(self.yaw_rad):.1f} deg, '
            f'hover={self.hover_seconds:.1f} s, rate={rate_hz:.1f} Hz')

    # @staticmethod
    def set_stage(self, stage):
        if self.stage != stage:
            self.stage = stage
            self.stage_started_at = self.get_clock().now()
            self.get_logger().info(f'Stage: {stage.name}')

    def stage_elapsed(self):
        delta = self.get_clock().now() - self.stage_started_at
        return delta.nanoseconds / 1e9
    
    def now_us(self):
        return self.get_clock().now().nanoseconds // 1000


    def position_callback(self, msg):
        self.position = msg

    def status_callback(self, msg):
        self.status = msg

    def land_callback(self, msg):
        self.land_detected = msg

    def ack_callback(self, msg):
        self.last_ack[msg.command] = msg.result
        if msg.command in (  
            # ？？？没看懂，为什么订阅VehicleCommandAck，但是要在msg.command中查询VehicleCommand
            # VehicleCommandAck主要就是监听VehicleCommand，通过读取command的值，对应到VehicleCommandAck的状态
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,             # 176
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,    # 400
            VehicleCommand.VEHICLE_CMD_NAV_LAND,                # 21
        ):
            self.get_logger().info(
                f'VehicleCommand ACK: command={msg.command}, result={msg.result}')

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_mode_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = float(self.yaw_rad)
        msg.yawspeed = math.nan
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        msg.from_external = True
        self.command_pub.publish(msg)

    def request_offboard(self):
        # PX4/MAVLink: param1=custom mode, param2=PX4 custom main mode.
        # PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.offboard_request_sent = True
        self.get_logger().info('Requested Offboard mode')

    def request_arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.arm_request_sent = True
        self.get_logger().info('Requested arm')

    def request_land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.land_sent = True
        self.get_logger().info('Requested land')

    def request_disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )
        self.disarm_request_sent = True
        self.get_logger().info('Disarm Shutdown offboard')

    def estimator_is_valid(self):
        return (
            self.position is not None
            and bool(self.position.xy_valid)
            and bool(self.position.z_valid)
        )

    def control_loop(self):
        # 任何阶段都必须持续发送这两个话题，否则 PX4 可能触发 Offboard 丢失保护。
        # /fmu/in/offboard_control_mode : publish_offboard_control_mode()
        # /fmu/in/Trajectory_setpoint : publish_position_setpoint(x, y, z)
        # 修改意见：在结束阶段，停止发送控制指令
        if self.stage != FlightStage.REQUEST_DISARM:
            self.publish_offboard_control_mode()

        # 飞行阶段： 等待
        if self.stage == FlightStage.WAIT_FOR_ESTIMATOR:
            if not self.estimator_is_valid():
                self.log_counter += 1
                if self.log_counter % 40 == 0:
                    self.get_logger().info('Waiting for valid local position...')
                return

            self.initial_x = float(self.position.x)
            self.initial_y = float(self.position.y)
            self.initial_z = float(self.position.z)
            self.get_logger().info(
                f'Initial position NED: x={self.initial_x:.2f}, '
                f'y={self.initial_y:.2f}, z={self.initial_z:.2f}')
            # 得到当前位置作为初始位置后，进入下一阶段：STREAM_SETPOINTS
            self.set_stage(FlightStage.STREAM_SETPOINTS)

        # 保持起飞目标；预发送阶段也使用同一个目标点。
        target_z = self.initial_z - self.target_height
        # 修改意见：在降落阶段，停止发送目标位置
        if self.stage != FlightStage.LAND:
            self.publish_position_setpoint(self.initial_x, self.initial_y, target_z)

        if self.stage == FlightStage.STREAM_SETPOINTS:
            #  这里就是offboard心跳：持续发送OffboardControlMode和TrajectorySetpoint至少1秒
            #  我们这里发送2秒，之后切换到下一阶段：REQUEST_OFFBOARD
            if self.stage_elapsed() >= 2.0:
                self.set_stage(FlightStage.REQUEST_OFFBOARD)

        elif self.stage == FlightStage.REQUEST_OFFBOARD:
            ## ？？？ 第一轮，可能到了2秒之后，VehicleStatus.nav_state还没有赋值，所以不能进入这个if判断
            # 这个导航模式VehicleStatus.NAVIGATION_STATE_OFFBOARD在哪赋值？
            # 这里检查导航状态nav_state，本质就是ros2给px4发布command后，
            # 双重确认：1.通过订阅CommandAck来确认Ack是否收到command；2.读取status，判断status中的某个状态是否改变
            if self.status is not None and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info('Offboard mode confirmed')
                self.set_stage(FlightStage.REQUEST_ARM)
            elif not self.offboard_request_sent or self.stage_elapsed() >= 1.0:
                # 请求offboard
                self.request_offboard()
        
        # 请求offboard完毕后，进入下一阶段：REQUEST_ARM
        elif self.stage == FlightStage.REQUEST_ARM:
            ## ？？再读取status中的ARMING_STATE_ARMED
            # # 这个解锁状态VehicleStatus.ARMING_STATE_ARMED在哪赋值？
            if self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info('Armed confirmed')
                self.set_stage(FlightStage.TAKEOFF)
            elif not self.arm_request_sent or self.stage_elapsed() >= 1.0:
                # 请求解锁
                self.request_arm()
 
        elif self.stage == FlightStage.TAKEOFF:
            if self.position is not None:
                height_error = abs(float(self.position.z) - target_z)
                if height_error < 0.25 and abs(float(self.position.vz)) < 0.5:
                    self.get_logger().info(
                        f'Takeoff target reached: z={self.position.z:.2f}')
                    self.hover_started_at = self.get_clock().now()
                    self.set_stage(FlightStage.HOVER)

        elif self.stage == FlightStage.HOVER:
            elapsed = (self.get_clock().now() - self.hover_started_at).nanoseconds / 1e9
            if elapsed >= self.hover_seconds:
                self.set_stage(FlightStage.LAND)

        elif self.stage == FlightStage.LAND:
            if not self.land_sent:
                self.request_land()
            if self.land_detected and self.land_detected.landed:
                self.get_logger().info('Landing detected....')
                # land后，如果PX4自动上锁，则直接结束
                if self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                    self.get_logger().info('Landed and disarmed')
                    self.set_stage(FlightStage.FINISHED)
                else:
                    self.set_stage(FlightStage.REQUEST_DISARM)
            # # 降落命令发送后仍继续发送设定点，直到 PX4 接管降落过程。
            # if self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            #     self.get_logger().info('Landed and disarmed')
            #     self.set_stage(FlightStage.FINISHED)
            # elif self.stage_elapsed() > 60.0:
            #     self.get_logger().error('Landing timeout after 60 seconds')
            #     self.set_stage(FlightStage.ERROR)

        elif self.stage == FlightStage.REQUEST_DISARM:
            if (self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED):
                self.get_logger().info('Disarm confirmed')
                self.set_stage(FlightStage.FINISHED)

            elif not self.disarm_request_sent or self.stage_elapsed() >= 1.0:
                self.request_disarm()
            
        elif self.stage == FlightStage.FINISHED:
            # 流程结束后不再发布新的 VehicleCommand。
            # pass
            exit()

        elif self.stage == FlightStage.ERROR:
            # pass
            exit()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PX4OffboardController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
