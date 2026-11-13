#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data


class RGBDetectorNode(Node):

    def __init__(self):

        super().__init__('rgb_detector_node')

        # ROS Image 转 OpenCV
        self.bridge = CvBridge()

        # RGB 图像订阅
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # 发布红色箱子的中心像素
        self.center_pub = self.create_publisher(
            PointStamped,
            '/red_box/center_pixel',
            10
        )

        self.get_logger().info(
            'RGB Detector Node Started'
        )

    def image_callback(self, msg):

        try:

            # ROS Image -> OpenCV
            image = self.bridge.imgmsg_to_cv2(
                msg,
                # ros2 topic echo /camera/image_raw --once
                # 看到的encoding:rgb8
                desired_encoding='bgr8'
            )

        except Exception as e:

            self.get_logger().error(
                f'Image conversion failed: {e}'
            )

            return

        # 获取图像尺寸
        height, width = image.shape[:2]

        # BGR -> HSV
        hsv = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2HSV
        )

        # 红色在 HSV 中跨越 0 度，需要设置两个范围
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # 创建红色掩膜
        mask1 = cv2.inRange(
            hsv,
            lower_red1,
            upper_red1
        )

        mask2 = cv2.inRange(
            hsv,
            lower_red2,
            upper_red2
        )

        mask = mask1 | mask2

        # 形态学处理，去除小噪声
        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # 查找轮廓
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:

            # 获取最大轮廓
            contour = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(contour)

            # 过滤小面积噪声
            if area > 100:

                x, y, w, h = cv2.boundingRect(
                    contour
                )

                # 红色箱子的中心像素
                u = x + w // 2
                v = y + h // 2

                # 绘制检测框
                cv2.rectangle(
                    image,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 0),
                    2
                )

                # 绘制中心点
                cv2.circle(
                    image,
                    (u, v),
                    5,
                    (255, 0, 0),
                    -1
                )

                cv2.putText(
                    image,
                    f'Red Box: ({u},{v})',
                    (x, max(y - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

                # 发布中心像素坐标
                point_msg = PointStamped()

                point_msg.header = msg.header

                point_msg.point.x = float(u)
                point_msg.point.y = float(v)
                point_msg.point.z = 0.0

                self.center_pub.publish(
                    point_msg
                )
        cv2.resizeWindow('RGB', 400, 300)
        cv2.resizeWindow('Mask', 800, 600)
        # 显示 RGB 图像
        cv2.imshow(
            'RGB',
            image
        )

        # 显示红色掩膜
        cv2.imshow(
            'Mask',
            mask
        )

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = RGBDetectorNode()

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