#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data


class DepthProcessorNode(Node):

    def __init__(self):

        super().__init__('depth_processor_node')

        self.bridge = CvBridge()

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

        # 红色箱子中心像素
        self.pixel_u = None
        self.pixel_v = None

        # 深度单位换算系数
        self.declare_parameter(
            'depth_scale_16uc1',
            0.001
        )

        self.depth_scale = float(
            self.get_parameter(
                'depth_scale_16uc1'
            ).value
        )

        # Depth Image
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            qos_profile_sensor_data
        )

        # Camera Info
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.rgb_info_callback,
            qos_profile_sensor_data
        )
        # Depth 相机内参
        self.depth_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.depth_info_callback,
            qos_profile_sensor_data
        )

        # 红色箱子中心像素
        self.pixel_sub = self.create_subscription(
            PointStamped,
            '/red_box/center_pixel',
            self.pixel_callback,
            10
        )

        self.get_logger().info(
            'Depth Processor Node Started'
        )

    # 获取相机内参
    def rgb_info_callback(self, msg):

        self.rgb_fx = msg.k[0]
        self.rgb_fy = msg.k[4]

        self.rgb_cx = msg.k[2]
        self.rgb_cy = msg.k[5]

        # self.get_logger().info(
        #     f'RGB Intrinsics: '
        #     f'fx={self.rgb_fx:.2f}, '
        #     f'fy={self.rgb_fy:.2f}, '
        #     f'cx={self.rgb_cx:.2f}, '
        #     f'cy={self.rgb_cy:.2f}'
        # )
    def depth_info_callback(self, msg):

        self.depth_fx = msg.k[0]
        self.depth_fy = msg.k[4]

        self.depth_cx = msg.k[2]
        self.depth_cy = msg.k[5]

        # self.get_logger().info(
        #     f'Depth Intrinsics: '
        #     f'fx={self.depth_fx:.2f}, '
        #     f'fy={self.depth_fy:.2f}, '
        #     f'cx={self.depth_cx:.2f}, '
        #     f'cy={self.depth_cy:.2f}'
        # )

    def rgb_to_depth_pixel(self, u_rgb, v_rgb):

        # 确保两组相机内参都已获取
        if (
            self.rgb_fx is None
            or self.depth_fx is None
        ):
            return None, None

        # RGB 像素 -> 归一化坐标
        x_norm = (
            u_rgb - self.rgb_cx
        ) / self.rgb_fx

        y_norm = (
            v_rgb - self.rgb_cy
        ) / self.rgb_fy

        # 归一化坐标 -> Depth 像素
        u_depth = (
            self.depth_fx * x_norm
            + self.depth_cx
        )

        v_depth = (
            self.depth_fy * y_norm
            + self.depth_cy
        )

        return int(round(u_depth)), int(round(v_depth))

    
    # 获取红色箱子中心像素
    def pixel_callback(self, msg):

        self.pixel_u = int(msg.point.x)
        self.pixel_v = int(msg.point.y)

    def depth_callback(self, msg):

        # 检查是否已经获得 RGB 检测结果
        if self.pixel_u is None:
            return

        # 检查两组相机内参
        if (
            self.rgb_fx is None
            or self.depth_fx is None
        ):
            return

        # ROS Image -> NumPy
        try:

            depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )

        except Exception as e:

            self.get_logger().error(
                f'Depth conversion failed: {e}'
            )

            return

        # 当前深度图是 32FC1，单位通常为米
        if msg.encoding == '32FC1':

            depth_image = depth_image.astype(
                np.float32
            )

        elif msg.encoding == '16UC1':

            depth_image = (
                depth_image.astype(np.float32)
                * self.depth_scale
            )

        else:

            self.get_logger().error(
                f'Unsupported encoding: {msg.encoding}'
            )

            return

        # RGB 像素映射到 Depth 像素
        u, v = self.rgb_to_depth_pixel(
            self.pixel_u,
            self.pixel_v
        )

        if u is None:
            return

        height, width = depth_image.shape

        # 检查是否越界
        if not (0 <= u < width and 0 <= v < height):

            self.get_logger().warning(
                f'Depth pixel out of bounds: '
                f'({u},{v}), '
                f'image size={width}x{height}'
            )
            exit()
            

        # 提取中心区域
        radius = 2

        u1 = max(0, u - radius)
        u2 = min(width, u + radius + 1)

        v1 = max(0, v - radius)
        v2 = min(height, v + radius + 1)

        roi = depth_image[v1:v2, u1:u2]

        # 过滤无效深度
        valid_depth = roi[
            np.isfinite(roi) & (roi > 0.0)
        ]

        if valid_depth.size == 0:

            self.get_logger().warning(
                'No valid depth at target position'
            )

            return

        # 中值滤波
        Z = float(np.median(valid_depth))

        # Depth 相机三维坐标
        X = (
            (u - self.depth_cx)
            * Z / self.depth_fx
        )

        Y = (
            (v - self.depth_cy)
            * Z / self.depth_fy
        )

        self.get_logger().info(
            f'Red Box Position: '
            # f'RGB=({self.pixel_u},{self.pixel_v}), '
            # f'Depth=({u},{v}), '
            f'X={X:.3f}m, '
            f'Y={Y:.3f}m, '
            f'Z={Z:.3f}m'
        )

        # 显示深度图
        display = np.nan_to_num(
            depth_image,
            nan=0.0,
            posinf=0.0,
            neginf=0.0
        )

        display = np.clip(
            display / 5.0 * 255.0,
            0,
            255
        ).astype(np.uint8)

        display = cv2.applyColorMap(
            display,
            cv2.COLORMAP_JET
        )

        cv2.circle(
            display,
            (u, v),
            5,
            (255, 255, 255),
            -1
        )

        cv2.imshow(
            'Depth Image',
            display
        )

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = DepthProcessorNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        cv2.destroyAllWindows()

        rclpy.shutdown()


if __name__ == '__main__':

    main()