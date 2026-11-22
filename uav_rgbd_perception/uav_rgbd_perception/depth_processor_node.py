#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PolygonStamped
from cv_bridge import CvBridge


class DepthProcessorNode(Node):
    """
    Keeps the original center-pixel RGB-D estimate for cross-checking.
    The full 3-D bounding box is produced by pointcloud_processor_node.
    RGB->Depth pixel mapping below is valid only for aligned/co-located RGB and depth optical axes.
    """
    def __init__(self):
        super().__init__('depth_processor_node')
        self.bridge = CvBridge()
        self.rgb_intrinsics = None
        self.depth_intrinsics = None
        self.center_uv = None
        self.last_center_stamp = None

        self.declare_parameter('depth_scale_16uc1', 0.001)
        self.declare_parameter('center_roi_radius', 4)
        self.declare_parameter('show_depth', True)
        self.depth_scale = float(self.get_parameter('depth_scale_16uc1').value)
        self.radius = int(self.get_parameter('center_roi_radius').value)
        self.show_depth = bool(self.get_parameter('show_depth').value)

        self.create_subscription(CameraInfo, '/camera/camera_info', self.rgb_info_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.depth_info_cb, qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/red_box/center_pixel', self.center_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(PolygonStamped, '/red_box/bbox', self.bbox_cb, 10)

        self.rgbd_pub = self.create_publisher(PointStamped, '/red_box/rgbd_position', 10)
        if self.show_depth:
            cv2.namedWindow('Depth Image', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Depth Image', 640, 480)
        self.get_logger().info('Depth Processor Node started')

    def rgb_info_cb(self, msg):
        self.rgb_intrinsics = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))

    def depth_info_cb(self, msg):
        self.depth_intrinsics = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))

    def center_cb(self, msg):
        self.center_uv = (int(round(msg.point.x)), int(round(msg.point.y)))
        self.last_center_stamp = msg.header.stamp

    def bbox_cb(self, msg):
        # Reserved for future ROI diagnostics; the authoritative box reconstruction uses mask + organized cloud.
        pass

    def map_rgb_to_depth(self, u, v):
        if self.rgb_intrinsics is None or self.depth_intrinsics is None:
            return None
        rfx, rfy, rcx, rcy = self.rgb_intrinsics
        dfx, dfy, dcx, dcy = self.depth_intrinsics
        if min(rfx, rfy, dfx, dfy) <= 0:
            return None
        xn = (u - rcx) / rfx
        yn = (v - rcy) / rfy
        return int(round(dfx * xn + dcx)), int(round(dfy * yn + dcy))

    def depth_cb(self, msg):
        if self.center_uv is None:
            return
        if self.rgb_intrinsics is None or self.depth_intrinsics is None:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().error(f'Depth conversion failed: {exc}')
            return

        if msg.encoding == '32FC1':
            depth = depth.astype(np.float32, copy=False)
        elif msg.encoding == '16UC1':
            depth = depth.astype(np.float32) * self.depth_scale
        else:
            self.get_logger().warning(f'Unsupported depth encoding: {msg.encoding}')
            return

        mapped = self.map_rgb_to_depth(*self.center_uv)
        if mapped is None:
            return
        u, v = mapped
        h, w = depth.shape
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().warning(f'Mapped depth pixel ({u},{v}) outside {w}x{h}; skip this frame')
            return

        x0, x1 = max(0, u-self.radius), min(w, u+self.radius+1)
        y0, y1 = max(0, v-self.radius), min(h, v+self.radius+1)
        roi = depth[y0:y1, x0:x1]
        valid = roi[np.isfinite(roi) & (roi > 0.05)]
        if valid.size == 0:
            return

        z = float(np.median(valid))
        dfx, dfy, dcx, dcy = self.depth_intrinsics
        # Pinhole back-projection gives optical coordinates:
        # X right, Y down, Z forward.
        x_opt = (u - dcx) * z / dfx
        y_opt = (v - dcy) * z / dfy
        z_opt = z

        # Gazebo's camera_link convention is commonly X forward, Y left, Z up.
        # Keep the output coordinates consistent with the PointCloud2 frame when
        # the incoming depth frame is camera_link. Experiment 7 will replace this
        # convention-specific conversion with an explicit TF transform.
        if msg.header.frame_id.endswith('camera_link'):
            xyz = (z_opt, -x_opt, -y_opt)
        else:
            xyz = (x_opt, y_opt, z_opt)

        out = PointStamped()
        out.header = msg.header
        out.point.x, out.point.y, out.point.z = map(float, xyz)
        self.rgbd_pub.publish(out)

        if self.show_depth:
            disp = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            disp = np.clip(disp / 8.0 * 255.0, 0, 255).astype(np.uint8)
            disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)
            cv2.circle(disp, (u, v), 5, (255, 255, 255), -1)
            cv2.imshow('Depth Image', disp)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_depth:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
