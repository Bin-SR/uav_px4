import math 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleStatus

class PX4StateMonitor(Node):
    def __init__(self):
        super().__init__('px4_state_monitor')

        # PX4 输出话题通常使用 Best Effort QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        # 保存无人机状态
        self.position = None
        self.attitude = None
        self.status = None


        self.position_sub = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback, 
            qos_profile)
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, 
            '/fmu/out/vehicle_attitude',
            self.attitude_callback, 
            qos_profile)
        
        self.status_sub = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status_v5',
            self.status_callback, 
            qos_profile)

        self.timer = self.create_timer(
            0.5,
            self.print_state
        )

        self.get_logger().info('P State Monitor started!!!!')

    def position_callback(self, msg):
        self.position = msg

    def attitude_callback(self, msg):
        self.attitude = msg

    def status_callback(self, msg):
        self.status = msg

    @staticmethod
    def quaternion_to_euler(q):
        w, x, y, z = q

        # Roll
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        # Yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    # -------------------------------
    # 枚举值转换为字符串
    # -------------------------------
    # 就是想把arming_state和nav_state这两种状态中每种枚举类型代表的状态，
    # 其中的枚举值变成对应的状态，以字符串形式打印出来，便于观察
    @staticmethod
    def enum_to_string(msg_class, prefix, value):

        for name in dir(msg_class):
            if name.startswith(prefix):
                if getattr(msg_class, name) == value:
                    return name.replace(prefix, '')

        return str(value)

    # -------------------------------
    # 打印状态
    # -------------------------------
    def print_state(self):

        # 等待数据全部到达
        if (self.position is None or self.attitude is None or self.status is None):
            self.get_logger().info(
                'Waiting for PX4 data...',
                throttle_duration_sec=3.0
            )
            return

        pos = self.position
        att = self.attitude
        status = self.status

        # 坐标系：NED
        roll, pitch, yaw = self.quaternion_to_euler(att.q)

        # 解锁状态
        arming_state = self.enum_to_string(
            VehicleStatus,
            'ARMING_STATE_',
            status.arming_state
        )

        # 飞行模式
        nav_state = self.enum_to_string(
            VehicleStatus,
            'NAVIGATION_STATE_',
            status.nav_state
        )

        # 清屏
        # print('\033[2J\033[H', end='')

        print('=' * 50)
        print('           PX4 UAV STATE MONITOR')
        print('=' * 50)

        print('\n[Position - NED]')

        print(
            f'Position: X={pos.x:8.3f} m  '
            f'Y={pos.y:8.3f} m  '
            f'Z={pos.z:8.3f} m'
        )

        print(
            f'Velocity: VX={pos.vx:7.3f} m/s  '
            f'VY={pos.vy:7.3f} m/s  '
            f'VZ={pos.vz:7.3f} m/s'
        )

        print(f'Heading:  {math.degrees(pos.heading):8.2f} deg')

        print('\n[Attitude]')

        print(f'Roll:     {roll:8.2f} deg')
        print(f'Pitch:    {pitch:8.2f} deg')
        print(f'Yaw:      {yaw:8.2f} deg')

        print('\n[Flight Status]')

        print(f'Arming:   {arming_state}')
        print(f'Nav Mode: {nav_state}')
        print(f'Failsafe: {status.failsafe}')

        print('\n[Validity]')

        print(f'XY Valid: {pos.xy_valid}')
        print(f'Z Valid:  {pos.z_valid}')
        print(f'Velocity XY Valid: {pos.v_xy_valid}')
        print(f'Velocity Z Valid:  {pos.v_z_valid}')

        print('\n' + '=' * 50)

def main(args=None):
    rclpy.init(args=args)
    node = PX4StateMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    
