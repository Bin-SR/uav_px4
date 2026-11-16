#!/usr/bin/env python3

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import (
    Image,
    CameraInfo,
    PointCloud2
)

from geometry_msgs.msg import PointStamped

from visualization_msgs.msg import Marker

from cv_bridge import CvBridge

from rclpy.qos import qos_profile_sensor_data

from sensor_msgs_py import point_cloud2


class PointCloudProcessorNode(Node):

    def __init__(self):

        super().__init__('pointcloud_processor_node')

        self.bridge = CvBridge()

        # ==========================================
        # 1. 相机内参
        # ==========================================

        # RGB 相机内参
        self.rgb_fx = None
        self.rgb_fy = None
        self.rgb_cx = None
        self.rgb_cy = None

        # Depth 相机内参
        self.depth_fx = None
        self.depth_fy = None
        self.depth_cx = None
        self.depth_cy = None

        self.rgb_frame_id = None
        self.depth_frame_id = None

        # ==========================================
        # 2. RGB 检测结果
        # ==========================================

        self.pixel_u = None
        self.pixel_v = None

        # ==========================================
        # 3. Depth 数据
        # ==========================================

        self.depth_image = None

        self.depth_scale = 0.001

        # ==========================================
        # 4. PointCloud 数据
        # ==========================================

        self.cloud_xyz = None

        self.cloud_width = 0
        self.cloud_height = 0

        self.cloud_frame_id = None
        self.cloud_header = None

        self.cloud_stamp = None
        self.depth_stamp = None

        # ==========================================
        # 5. 参数
        # ==========================================

        self.declare_parameter(
            'roi_radius',
            4
        )

        self.declare_parameter(
            'voxel_size',
            0.04
        )

        self.declare_parameter(
            'depth_scale_16uc1',
            0.001
        )

        self.declare_parameter(
            'visualize',
            True
        )

        self.roi_radius = int(
            self.get_parameter(
                'roi_radius'
            ).value
        )

        self.voxel_size = float(
            self.get_parameter(
                'voxel_size'
            ).value
        )

        self.depth_scale = float(
            self.get_parameter(
                'depth_scale_16uc1'
            ).value
        )

        self.visualize = bool(
            self.get_parameter(
                'visualize'
            ).value
        )

        # ==========================================
        # 6. ROS Subscribers
        # ==========================================

        self.rgb_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.rgb_info_callback,
            qos_profile_sensor_data
        )

        self.depth_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.depth_info_callback,
            qos_profile_sensor_data
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            qos_profile_sensor_data
        )

        self.pixel_sub = self.create_subscription(
            PointStamped,
            '/red_box/center_pixel',
            self.pixel_callback,
            10
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.cloud_callback,
            qos_profile_sensor_data
        )

        # ==========================================
        # 7. ROS Publishers
        # ==========================================

        self.cloud_point_pub = self.create_publisher(
            PointStamped,
            '/red_box/pointcloud_position',
            10
        )

        self.rgbd_point_pub = self.create_publisher(
            PointStamped,
            '/red_box/rgbd_position',
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/red_box/pointcloud_marker',
            10
        )

        # ==========================================
        # 8. Open3D 初始化
        # ==========================================

        self.vis = None

        self.pcd = None

        self.geometry_added = False

        self.pending_visualization = False

        if self.visualize:

            try:

                self.vis = o3d.visualization.Visualizer()

                self.vis.create_window(
                    window_name='PointCloud2 - UAV RGBD',
                    width=960,
                    height=720
                )

                self.pcd = o3d.geometry.PointCloud()

                self.get_logger().info(
                    'Open3D visualization initialized'
                )

            except Exception as e:

                self.get_logger().warning(
                    f'Open3D initialization failed: {e}'
                )

                self.vis = None

        # Open3D 窗口更新定时器
        self.visualization_timer = self.create_timer(
            0.03,
            self.visualization_callback
        )

        # 限制日志频率
        self.last_log_time = 0.0

        self.get_logger().info(
            'PointCloud Processor Node Started'
        )

    # ==================================================
    # RGB CameraInfo
    # ==================================================

    def rgb_info_callback(self, msg):

        self.rgb_fx = msg.k[0]
        self.rgb_fy = msg.k[4]

        self.rgb_cx = msg.k[2]
        self.rgb_cy = msg.k[5]

        self.rgb_frame_id = msg.header.frame_id

        # self.get_logger().info(
        #     f'RGB CameraInfo received: '
        #     f'{msg.width}x{msg.height}, '
        #     f'fx={self.rgb_fx:.2f}, '
        #     f'fy={self.rgb_fy:.2f}'
        # )

    # ==================================================
    # Depth CameraInfo
    # ==================================================

    def depth_info_callback(self, msg):

        self.depth_fx = msg.k[0]
        self.depth_fy = msg.k[4]

        self.depth_cx = msg.k[2]
        self.depth_cy = msg.k[5]

        self.depth_frame_id = msg.header.frame_id

        # self.get_logger().info(
        #     f'Depth CameraInfo received: '
        #     f'{msg.width}x{msg.height}, '
        #     f'fx={self.depth_fx:.2f}, '
        #     f'fy={self.depth_fy:.2f}'
        # )

    # ==================================================
    # RGB Detector
    # ==================================================

    def pixel_callback(self, msg):

        self.pixel_u = int(msg.point.x)
        self.pixel_v = int(msg.point.y)

    # ==================================================
    # Depth Image
    # ==================================================

    def depth_callback(self, msg):

        try:

            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )

        except Exception as e:

            self.get_logger().error(
                f'Depth conversion failed: {e}'
            )

            return

        if msg.encoding == '32FC1':

            image = image.astype(np.float32)

        elif msg.encoding == '16UC1':

            image = (
                image.astype(np.float32)
                * self.depth_scale
            )

        else:

            self.get_logger().error(
                f'Unsupported depth encoding: {msg.encoding}'
            )

            return

        self.depth_image = image

        self.depth_stamp = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

    # ==================================================
    # RGB Pixel -> Depth Pixel
    # ==================================================

    def rgb_to_depth_pixel(self, u_rgb, v_rgb):

        if (
            self.rgb_fx is None
            or self.depth_fx is None
        ):

            return None, None

        # RGB 像素坐标转换为归一化坐标
        x_norm = (
            u_rgb - self.rgb_cx
        ) / self.rgb_fx

        y_norm = (
            v_rgb - self.rgb_cy
        ) / self.rgb_fy

        # 归一化坐标转换到 Depth 图像
        u_depth = (
            self.depth_fx * x_norm
            + self.depth_cx
        )

        v_depth = (
            self.depth_fy * y_norm
            + self.depth_cy
        )

        return (
            int(round(u_depth)),
            int(round(v_depth))
        )

    # ==================================================
    # PointCloud2 -> NumPy
    # ==================================================

    def pointcloud_to_numpy(self, msg):

        points = point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=False
        )

        # 兼容不同版本 sensor_msgs_py
        points = np.asarray(points)

        if points.dtype.names is not None:

            x = points['x']
            y = points['y']
            z = points['z']

            xyz = np.column_stack(
                (x, y, z)
            )

        else:

            xyz = np.asarray(
                list(points),
                dtype=np.float32
            )

        xyz = xyz.astype(
            np.float32,
            copy=False
        )

        return xyz

    # ==================================================
    # PointCloud2 Callback
    # ==================================================

    def cloud_callback(self, msg):

        try:

            xyz = self.pointcloud_to_numpy(msg)

        except Exception as e:

            self.get_logger().error(
                f'PointCloud2 conversion failed: {e}'
            )

            return

        # 保存点云信息
        self.cloud_width = msg.width
        self.cloud_height = msg.height

        self.cloud_frame_id = msg.header.frame_id

        self.cloud_header = msg.header

        self.cloud_stamp = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        # 检查点数
        if xyz.size == 0:

            self.get_logger().warning(
                'Empty PointCloud2'
            )

            return

        # 检查点云组织形式
        if xyz.shape[0] != msg.width * msg.height:

            self.get_logger().warning(
                'PointCloud2 data size mismatch'
            )

            return

        self.cloud_xyz = xyz.reshape(
            msg.height,
            msg.width,
            3
        )

        # Open3D 可视化
        if self.vis is not None:

            self.update_visualization(xyz)

        # 提取障碍物点云
        self.process_target()

    # ==================================================
    # Open3D Visualization
    # ==================================================

    def update_visualization(self, xyz):

        valid = np.isfinite(xyz).all(axis=1)

        valid_points = xyz[valid]

        if valid_points.shape[0] == 0:
            return

        # 创建临时点云
        temp_pcd = o3d.geometry.PointCloud()

        temp_pcd.points = o3d.utility.Vector3dVector(
            valid_points
        )

        # 降采样，减少显示压力
        temp_pcd = temp_pcd.voxel_down_sample(
            voxel_size=self.voxel_size
        )

        self.pending_points = np.asarray(
            temp_pcd.points
        )

        self.pending_visualization = True

    def visualization_callback(self):

        if self.vis is None:
            return

        if self.pending_visualization:

            points = self.pending_points

            self.pcd.points = o3d.utility.Vector3dVector(
                points
            )

            if not self.geometry_added:

                self.vis.add_geometry(
                    self.pcd
                )

                self.geometry_added = True

            else:

                self.vis.update_geometry(
                    self.pcd
                )

            self.pending_visualization = False

        try:

            self.vis.poll_events()

            self.vis.update_renderer()

        except Exception as e:

            self.get_logger().warning(
                f'Open3D update failed: {e}'
            )

            self.vis = None

    # ==================================================
    # RGB-D Method
    # ==================================================

    def calculate_rgbd_position(self, u, v):

        if self.depth_image is None:
            return None

        height, width = self.depth_image.shape

        if not (
            0 <= u < width
            and 0 <= v < height
        ):

            return None

        r = self.roi_radius

        u1 = max(0, u - r)
        u2 = min(width, u + r + 1)

        v1 = max(0, v - r)
        v2 = min(height, v + r + 1)

        roi = self.depth_image[
            v1:v2,
            u1:u2
        ]

        valid = roi[
            np.isfinite(roi)
            & (roi > 0.0)
        ]

        if valid.size == 0:
            return None

        Z = float(np.median(valid))

        X = (
            (u - self.depth_cx)
            * Z / self.depth_fx
        )

        Y = (
            (v - self.depth_cy)
            * Z / self.depth_fy
        )

        return np.array(
            [X, Y, Z],
            dtype=np.float32
        )

    # ==================================================
    # PointCloud2 Target Extraction
    # ==================================================

    def extract_target_points(self, u, v):

        if self.cloud_xyz is None:
            return None

        height, width, _ = self.cloud_xyz.shape

        # 确保点云与 Depth 图像分辨率一致
        if self.depth_image is None:
            return None

        depth_height, depth_width = self.depth_image.shape

        if (
            width != depth_width
            or height != depth_height
        ):

            self.get_logger().warning(
                'PointCloud2 and Depth Image dimensions '
                'do not match. Cannot directly index cloud.'
            )

            return None

        r = self.roi_radius

        u1 = max(0, u - r)
        u2 = min(width, u + r + 1)

        v1 = max(0, v - r)
        v2 = min(height, v + r + 1)

        roi = self.cloud_xyz[
            v1:v2,
            u1:u2,
            :
        ]

        points = roi.reshape(-1, 3)

        # 过滤无效点
        valid = (
            np.isfinite(points).all(axis=1)
            & (points[:, 2] > 0.0)
        )

        points = points[valid]

        if points.shape[0] == 0:
            return None

        return points

    # ==================================================
    # Publish Point
    # ==================================================

    def publish_point(self, point, topic_type):

        msg = PointStamped()

        msg.header = self.cloud_header

        msg.point.x = float(point[0])
        msg.point.y = float(point[1])
        msg.point.z = float(point[2])

        if topic_type == 'cloud':

            self.cloud_point_pub.publish(msg)

        elif topic_type == 'rgbd':

            self.rgbd_point_pub.publish(msg)

    # ==================================================
    # Publish Marker
    # ==================================================

    def publish_marker(self, point):

        marker = Marker()

        marker.header = self.cloud_header

        marker.ns = 'red_box'
        marker.id = 0

        marker.type = Marker.SPHERE

        marker.action = Marker.ADD

        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = float(point[2])

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0

        self.marker_pub.publish(marker)

    # ==================================================
    # Main Target Processing
    # ==================================================

    def process_target(self):

        # 检查必要数据
        if (
            self.pixel_u is None
            or self.cloud_xyz is None
            or self.depth_image is None
            or self.rgb_fx is None
            or self.depth_fx is None
        ):

            return

        # 检查坐标系
        if (
            self.depth_frame_id
            and self.cloud_frame_id
            and self.depth_frame_id != self.cloud_frame_id
        ):

            self.get_logger().warning(
                f'Frame mismatch: '
                f'Depth={self.depth_frame_id}, '
                f'Cloud={self.cloud_frame_id}. '
                f'Please transform before comparison.'
            )

            return

        # RGB 像素 -> Depth 像素
        u, v = self.rgb_to_depth_pixel(
            self.pixel_u,
            self.pixel_v
        )

        if u is None:
            return

        height, width = self.depth_image.shape

        if not (
            0 <= u < width
            and 0 <= v < height
        ):
            return

        # ==========================================
        # 方法1：RGB-D
        # ==========================================

        rgbd_point = self.calculate_rgbd_position(
            u, v
        )

        if rgbd_point is None:
            return

        # ==========================================
        # 方法2：PointCloud2
        # ==========================================

        target_points = self.extract_target_points(
            u, v
        )

        if target_points is None:
            return

        # 对目标区域点云取中值
        cloud_point = np.median(
            target_points,
            axis=0
        )

        # ==========================================
        # 坐标比较
        # ==========================================

        error = np.linalg.norm(
            cloud_point - rgbd_point
        )

        # 发布坐标
        self.publish_point(
            cloud_point,
            'cloud'
        )

        self.publish_point(
            rgbd_point,
            'rgbd'
        )

        self.publish_marker(
            cloud_point
        )

        # ==========================================
        # 输出日志
        # ==========================================

        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_log_time >= 1.0:

            self.get_logger().info(
                '\n========== RGB-D vs PointCloud ==========\n'
                f'RGB Pixel: '
                f'({self.pixel_u}, {self.pixel_v})\n'
                f'Depth Pixel: ({u}, {v})\n'
                f'Cloud Points: {len(target_points)}\n\n'

                f'RGB-D Position:\n'
                f'X={rgbd_point[0]:.3f}, '
                f'Y={rgbd_point[1]:.3f}, '
                f'Z={rgbd_point[2]:.3f}\n\n'

                f'PointCloud Position:\n'
                f'X={cloud_point[0]:.3f}, '
                f'Y={cloud_point[1]:.3f}, '
                f'Z={cloud_point[2]:.3f}\n\n'

                f'Euclidean Error: {error:.4f} m\n'
                '========================================='
            )

            self.last_log_time = now

    # ==================================================
    # Shutdown
    # ==================================================

    def destroy_node(self):

        if self.vis is not None:

            self.vis.destroy_window()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = PointCloudProcessorNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()