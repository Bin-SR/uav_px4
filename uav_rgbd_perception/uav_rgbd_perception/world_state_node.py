#!/usr/bin/env python3
"""
Fuse the existing camera-frame AABB and PX4 state into a JSON World State.

Assumptions for this first Gazebo x500_depth experiment:
- PointCloud2 and /red_box/position are expressed in camera_link (Gazebo FLU:
  +X forward, +Y left, +Z up), as indicated by the current pipeline.
- PX4 VehicleOdometry pose_frame=1 is NED and its body convention is FRD.
- x500_depth SDF camera_link is fixed to base_link at (0.12, 0.03, 0.242)
  in Gazebo FLU. The camera_link axes are parallel to base_link axes.
- The camera-to-body translation and axes are converted from FLU to FRD below.

The node transforms all 8 camera-AABB corners, then recomputes a world ENU AABB.
"""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import String, Float32
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry


S_NED_TO_ENU = np.array([[0.0, 1.0, 0.0],
                         [1.0, 0.0, 0.0],
                         [0.0, 0.0, -1.0]], dtype=np.float64)
D_FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def quat_wxyz_to_matrix(q):
    """PX4 quaternion [w, x, y, z], returning the represented rotation matrix."""
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid odometry quaternion')
    w, x, y, z = q / norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ], dtype=np.float64)


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class WorldStateNode(Node):
    def __init__(self):
        super().__init__('world_state_node')
        self.declare_parameter('position_topic', '/red_box/position')
        self.declare_parameter('size_topic', '/red_box/size')
        self.declare_parameter('local_position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('odometry_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('world_state_topic', '/uav/world_state')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('world_frame', 'map_enu')
        self.declare_parameter('camera_translation_flu', [0.12, 0.03, 0.242])
        self.declare_parameter('object_class', 'sports ball')
        self.declare_parameter('object_confidence', 0.0)
        self.declare_parameter('object_timeout_sec', 1.5)
        self.declare_parameter('publish_hz', 5.0)
        self.declare_parameter('pose_frame_ned_value', 1)

        p = lambda name: self.get_parameter(name).value
        self.camera_frame = str(p('camera_frame'))
        self.world_frame = str(p('world_frame'))
        self.camera_t_flu = np.asarray(p('camera_translation_flu'), dtype=np.float64)
        self.object_class = str(p('object_class'))
        self.object_confidence = float(p('object_confidence'))
        self.object_timeout = float(p('object_timeout_sec'))
        self.pose_frame_ned_value = int(p('pose_frame_ned_value'))

        self.local_msg = None
        self.odom_msg = None
        self.position_msg = None
        self.size_msg = None
        self.confidence = 0.0
        self.position_rx = 0.0
        self.size_rx = 0.0

        self.create_subscription(
            VehicleLocalPosition, str(p('local_position_topic')),
            self.local_cb, qos_profile_sensor_data)
        self.create_subscription(
            VehicleOdometry, str(p('odometry_topic')),
            self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(
            PointStamped, str(p('position_topic')), self.position_cb, 10)
        self.create_subscription(
            Vector3Stamped, str(p('size_topic')), self.size_cb, 10)
        self.create_subscription(
            Float32, '/red_box/confidence', self.confidence_cb, 10)
        self.pub = self.create_publisher(String, str(p('world_state_topic')), 10)
        hz = max(0.2, float(p('publish_hz')))
        self.timer = self.create_timer(1.0 / hz, self.publish_state)
        self.get_logger().info(
            f'World State node started: {p("local_position_topic")}, '
            f'{p("odometry_topic")} -> {p("world_state_topic")}')

    def local_cb(self, msg):
        self.local_msg = msg

    def odom_cb(self, msg):
        self.odom_msg = msg

    def position_cb(self, msg):
        self.position_msg = msg
        self.position_rx = time.monotonic()

    def size_cb(self, msg):
        self.size_msg = msg
        self.size_rx = time.monotonic()

    def confidence_cb(self, msg):
        self.confidence = float(msg.data)

    def get_drone_state(self):
        if self.local_msg is None:
            return None
        m = self.local_msg
        pos_ned = np.array([m.x, m.y, m.z], dtype=np.float64)
        vel_ned = np.array([m.vx, m.vy, m.vz], dtype=np.float64)
        pos_enu = S_NED_TO_ENU @ pos_ned
        vel_enu = S_NED_TO_ENU @ vel_ned
        yaw_enu = wrap_pi(math.pi / 2.0 - float(m.heading))
        return {
            'position_world': pos_enu.tolist(),
            'velocity_world': vel_enu.tolist(),
            'yaw': yaw_enu,
            'source_frame': 'PX4_LOCAL_NED',
            'valid': bool(m.xy_valid and m.z_valid and m.v_xy_valid and m.v_z_valid)
        }

    def camera_to_world(self):
        """Return camera_link FLU -> world ENU rotation and camera origin in ENU."""
        if self.odom_msg is None:
            return None
        m = self.odom_msg
        if int(m.pose_frame) != self.pose_frame_ned_value:
            raise ValueError(
                f'Expected PX4 pose_frame=NED({self.pose_frame_ned_value}), got {m.pose_frame}')

        p_ned = np.asarray(m.position, dtype=np.float64)
        r_ned_frd = quat_wxyz_to_matrix(m.q)
        p_enu = S_NED_TO_ENU @ p_ned
        r_enu_frd = S_NED_TO_ENU @ r_ned_frd

        # SDF translation is expressed in Gazebo body FLU. Convert it to PX4 FRD.
        t_body_frd = D_FLU_TO_FRD @ self.camera_t_flu
        camera_origin_enu = p_enu + r_enu_frd @ t_body_frd

        # camera_link is parallel to Gazebo base_link (FLU); convert camera FLU axes to FRD.
        r_enu_camera_flu = r_enu_frd @ D_FLU_TO_FRD
        return r_enu_camera_flu, camera_origin_enu

    @staticmethod
    def aabb_corners(center, size):
        c = np.asarray(center, dtype=np.float64)
        half = np.asarray(size, dtype=np.float64) * 0.5
        return np.array([
            [c[0] + sx*half[0], c[1] + sy*half[1], c[2] + sz*half[2]]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ], dtype=np.float64)

    def make_object(self):
        if self.position_msg is None or self.size_msg is None:
            return None
        now = time.monotonic()
        if now - self.position_rx > self.object_timeout or now - self.size_rx > self.object_timeout:
            return None
        if self.position_msg.header.frame_id not in ('', self.camera_frame):
            raise ValueError(
                f"Object position frame is '{self.position_msg.header.frame_id}', "
                f"expected '{self.camera_frame}'")
        if self.size_msg.header.frame_id not in ('', self.camera_frame):
            raise ValueError(
                f"Object size frame is '{self.size_msg.header.frame_id}', "
                f"expected '{self.camera_frame}'")

        center_cam = np.array([
            self.position_msg.point.x,
            self.position_msg.point.y,
            self.position_msg.point.z
        ], dtype=np.float64)
        size_cam = np.array([
            self.size_msg.vector.x,
            self.size_msg.vector.y,
            self.size_msg.vector.z
        ], dtype=np.float64)
        if not np.isfinite(center_cam).all() or not np.isfinite(size_cam).all():
            return None
        if np.any(size_cam <= 0.0):
            return None

        transform = self.camera_to_world()
        if transform is None:
            return None
        r_world_camera, t_world_camera = transform

        corners_cam = self.aabb_corners(center_cam, size_cam)
        corners_world = (r_world_camera @ corners_cam.T).T + t_world_camera
        lower = np.min(corners_world, axis=0)
        upper = np.max(corners_world, axis=0)
        center_world = 0.5 * (lower + upper)
        size_world = upper - lower

        return {
            'id': 1,
            'class': self.object_class,
            'confidence': self.confidence if self.confidence > 0.0 else self.object_confidence,
            'position_camera': center_cam.tolist(),
            'size_camera': size_cam.tolist(),
            'position_world': center_world.tolist(),
            'size_world': size_world.tolist(),
            'bbox_type': 'AABB',
            'geometry_source': 'visible_surface_pointcloud_percentile_aabb',
            'coordinate_frame': self.world_frame,
            'camera_frame': self.camera_frame,
            'valid': True
        }

    def publish_state(self):
        drone = self.get_drone_state()
        if drone is None:
            return
        state = {
            'timestamp_unix': time.time(),
            'coordinate_frame': self.world_frame,
            'drone': drone,
            'objects': []
        }
        try:
            obj = self.make_object()
            if obj is not None:
                state['objects'].append(obj)
        except Exception as exc:
            self.get_logger().warning(f'World object transform skipped: {exc}')
        msg = String()
        msg.data = json.dumps(state, ensure_ascii=False, separators=(',', ':'))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WorldStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
