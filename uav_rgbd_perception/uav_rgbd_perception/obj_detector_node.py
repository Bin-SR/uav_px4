#!/usr/bin/env python3
"""
YOLO-based RGB detector for the UAV RGB-D experiment.

Publishes the same interfaces consumed by the existing depth and point-cloud nodes:
  /red_box/mask          sensor_msgs/Image (mono8, RGB image resolution)
  /red_box/bbox          geometry_msgs/PolygonStamped (pixel corners)
  /red_box/center_pixel  geometry_msgs/PointStamped (pixel center)

The historical /red_box topic names are retained for compatibility. The detected
object is selected by COCO class name (default: sports ball), not by HSV color.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped
from cv_bridge import CvBridge
from ultralytics import YOLO


class ObjDetectorNode(Node):
    def __init__(self):
        super().__init__('obj_detector_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('target_class', 'sports ball')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('process_hz', 8.0)
        self.declare_parameter('show_window', False)
        self.declare_parameter('device', '')  # empty = Ultralytics automatic selection

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.target_class = str(self.get_parameter('target_class').value)
        self.conf = float(self.get_parameter('confidence_threshold').value)
        self.iou = float(self.get_parameter('iou_threshold').value)
        self.imgsz = int(self.get_parameter('image_size').value)
        self.period = 1.0 / max(0.1, float(self.get_parameter('process_hz').value))
        self.show_window = bool(self.get_parameter('show_window').value)
        self.device = str(self.get_parameter('device').value)

        model_path = str(self.get_parameter('model_path').value)
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        names = self.model.names
        if isinstance(names, dict):
            matches = [int(k) for k, v in names.items()
                       if str(v).strip().lower() == self.target_class.lower()]
        else:
            matches = [i for i, v in enumerate(names)
                       if str(v).strip().lower() == self.target_class.lower()]
        if not matches:
            raise RuntimeError(
                f"Class '{self.target_class}' not found in model names. "
                f"Available classes include: {list(names.values())[:10] if isinstance(names, dict) else list(names)[:10]}")
        self.class_id = matches[0]

        self.bridge = CvBridge()
        self.last_process_time = 0.0
        self.mask_pub = self.create_publisher(Image, '/red_box/mask', qos_profile_sensor_data)
        self.bbox_pub = self.create_publisher(PolygonStamped, '/red_box/bbox', 10)
        self.center_pub = self.create_publisher(PointStamped, '/red_box/center_pixel', 10)
        self.confidence_pub = self.create_publisher(Float32, '/red_box/confidence', 10)
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'YOLO detector ready: class={self.target_class}, id={self.class_id}, '
            f'conf={self.conf:.2f}, image_topic={self.image_topic}')

    def image_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_process_time < self.period:
            return
        self.last_process_time = now

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'RGB conversion failed: {exc}')
            return

        h, w = image.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        vis = image.copy()

        try:
            kwargs = dict(source=image, 
                          # conf=self.conf,
                          conf=0.10, 
                          iou=self.iou,
                          imgsz=self.imgsz, 
                          classes=[self.class_id],
                          verbose=False, max_det=10)
            if self.device:
                kwargs['device'] = self.device
            result = self.model.predict(**kwargs)[0]
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return


        # Debug: print all detected classes
        if result.boxes is not None and len(result.boxes) > 0:

            boxes = result.boxes

            scores = boxes.conf.detach().cpu().numpy()
            ids = boxes.cls.detach().cpu().numpy().astype(int)

            names = self.model.names

            detections = []

            for cls_id, score in zip(ids, scores):

                class_name = names[int(cls_id)]

                detections.append(
                    f"{class_name}: {score:.3f}"
                )

            self.get_logger().info(
                "YOLO detections: " + ", ".join(detections)
            )

        else:

            self.get_logger().info(
                "YOLO: no objects detected"
            )


        best = None
        if result.boxes is not None and len(result.boxes) > 0:
            boxes = result.boxes
            scores = boxes.conf.detach().cpu().numpy()
            ids = boxes.cls.detach().cpu().numpy().astype(int)
            candidates = np.where(ids == self.class_id)[0]
            if candidates.size:
                idx = int(candidates[np.argmax(scores[candidates])])
                xyxy = boxes.xyxy[idx].detach().cpu().numpy()
                x1, y1, x2, y2 = xyxy.tolist()
                x1 = int(np.clip(np.floor(x1), 0, w - 1))
                y1 = int(np.clip(np.floor(y1), 0, h - 1))
                x2 = int(np.clip(np.ceil(x2), 0, w))
                y2 = int(np.clip(np.ceil(y2), 0, h))
                if x2 > x1 and y2 > y1:
                    best = (x1, y1, x2, y2, float(scores[idx]))

        if best is not None:
            x1, y1, x2, y2, score = best
            # A rectangular detection mask is intentional for this first detector.
            # The downstream organized-cloud clustering removes much of the background.
            mask[y1:y2, x1:x2] = 255

            bbox = PolygonStamped()
            bbox.header = msg.header
            bbox.polygon.points = [
                Point32(x=float(x1), y=float(y1), z=0.0),
                Point32(x=float(x2), y=float(y1), z=0.0),
                Point32(x=float(x2), y=float(y2), z=0.0),
                Point32(x=float(x1), y=float(y2), z=0.0),
            ]
            self.bbox_pub.publish(bbox)

            center = PointStamped()
            center.header = msg.header
            center.point.x = 0.5 * (x1 + x2)
            center.point.y = 0.5 * (y1 + y2)
            center.point.z = 0.0
            self.center_pub.publish(center)
            confidence_msg = Float32()
            confidence_msg.data = score
            self.confidence_pub.publish(confidence_msg)

            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(vis, f'{self.target_class} {score:.2f}',
                        (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2)
        else:
            # Publish an empty mask every processed frame so the point-cloud node
            # does not keep treating an old detection as current.
            empty_bbox = PolygonStamped()
            empty_bbox.header = msg.header
            self.bbox_pub.publish(empty_bbox)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        if self.show_window:
            cv2.imshow('YOLO Object Detection', cv2.resize(vis, (960, 540)))
            cv2.imshow('YOLO Detection Mask', cv2.resize(mask, (960, 540)))
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObjDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f'obj_detector_node startup failed: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
