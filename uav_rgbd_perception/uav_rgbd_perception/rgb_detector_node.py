#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PolygonStamped, Point32
from cv_bridge import CvBridge


class RGBDetectorNode(Node):
    def __init__(self):
        super().__init__('rgb_detector_node')
        self.bridge = CvBridge()

        self.declare_parameter('min_contour_area', 150.0)
        self.declare_parameter('show_windows', True)
        self.declare_parameter('publish_largest_only', True)

        self.min_area = float(self.get_parameter('min_contour_area').value)
        self.show_windows = bool(self.get_parameter('show_windows').value)
        self.publish_largest_only = bool(self.get_parameter('publish_largest_only').value)

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)

        self.center_pub = self.create_publisher(PointStamped, '/red_box/center_pixel', 10)
        self.bbox_pub = self.create_publisher(PolygonStamped, '/red_box/bbox', 10)
        self.mask_pub = self.create_publisher(Image, '/red_box/mask', qos_profile_sensor_data)

        if self.show_windows:
            cv2.namedWindow('RGB Detection', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Red Mask', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('RGB Detection', 960, 540)
            cv2.resizeWindow('Red Mask', 640, 360)

        self.get_logger().info('RGB Detector Node started: publishing center_pixel, bbox and mask')

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'RGB conversion failed: {exc}')
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower1 = np.array([0, 60, 50], dtype=np.uint8)
        upper1 = np.array([10, 255, 255], dtype=np.uint8)
        lower2 = np.array([160, 60, 50], dtype=np.uint8)
        upper2 = np.array([180, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) >= self.min_area]
        if valid:
            contour = max(valid, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour)
            # Pixel coordinates use inclusive image indices; right/bottom are x+w-1,y+h-1.
            x1, y1 = x, y
            x2, y2 = x + w - 1, y + h - 1
            u, v = (x1 + x2) // 2, (y1 + y2) // 2

            center = PointStamped()
            center.header = msg.header
            center.point.x = float(u)
            center.point.y = float(v)
            center.point.z = 0.0
            self.center_pub.publish(center)

            bbox = PolygonStamped()
            bbox.header = msg.header
            bbox.polygon.points = [
                Point32(x=float(x1), y=float(y1), z=0.0),  # top-left
                Point32(x=float(x2), y=float(y1), z=0.0),  # top-right
                Point32(x=float(x2), y=float(y2), z=0.0),  # bottom-right
                Point32(x=float(x1), y=float(y2), z=0.0),  # bottom-left
            ]
            self.bbox_pub.publish(bbox)

            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(image, (u, v), 5, (255, 0, 0), -1)
            cv2.putText(image, f'Red box ({u},{v}) area={cv2.contourArea(contour):.0f}',
                        (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 0), 2)

        # Publish the complete binary mask every frame, including an empty mask when no target exists.
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)
        except Exception as exc:
            self.get_logger().error(f'Mask publication failed: {exc}')

        if self.show_windows:
            cv2.imshow('RGB Detection', image)
            cv2.imshow('Red Mask', mask)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_windows:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RGBDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
