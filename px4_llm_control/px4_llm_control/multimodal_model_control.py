"""VLM perception and navigation-intent node. Does not publish PX4 commands."""
import json
import math
import os
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor

from PIL import ImageDraw
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

from .multimodal_core import (parse_task, rgb_image, depth_array, TargetLock, project_box,
                              depth_front_distance, cloud_front_distance)
from .vlm_backends import OpenAIVisionBackend, RexOmniBackend


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9


class MultimodalModelControl(Node):
    def __init__(self):
        super().__init__('multimodal_model_control')
        defaults = {
            'backend': 'openai', 'base_url': 'http://127.0.0.1:8000/v1',
            'model': 'Qwen/Qwen2.5-VL-7B-Instruct', 'api_timeout': 20.0, 'max_tokens': 2048,
            'rex_model_path': 'IDEA-Research/Rex-Omni', 'rex_backend': 'transformers',
            'rgb_topic': '/camera/image_raw', 'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera_info', 'scan_topic': '/lidar/scan',
            'points_topic': '/lidar/points', 'inference_period': 1.0, 'max_image_side': 768,
            'sensor_timeout': 1.0, 'max_result_age': 3.0, 'sync_slop': 0.1,
            'depth_registered': False, 'images_rectified': False,
            'optical_frame': '', 'world_frame': 'map', 'required_obstacle_sources': ['depth'],
            'depth_forward_facing': False, 'scan_frame': 'lidar_link',
            'points_frame': 'base_link', 'scan_forward_angle_deg': 0.0,
            'scan_half_angle_deg': 30.0, 'stop_distance': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        for name in ('api_timeout', 'inference_period', 'sensor_timeout', 'max_result_age',
                     'sync_slop', 'stop_distance'):
            if not math.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError(f'{name}必须为正的有限数值')
        if self.cfg['max_image_side'] < 64 or self.cfg['max_tokens'] < 1:
            raise ValueError('max_image_side至少64，max_tokens为正整数')
        if not set(self.cfg['required_obstacle_sources']) <= {'depth', 'scan', 'points'}:
            raise ValueError('required_obstacle_sources只支持depth/scan/points')
        if not 0 < self.cfg['scan_half_angle_deg'] <= 180:
            raise ValueError('scan_half_angle_deg范围(0,180]')
        if self.cfg['backend'] == 'openai':
            self.backend = OpenAIVisionBackend(self.cfg['base_url'], self.cfg['model'],
                os.environ.get('VLM_API_KEY', ''), self.cfg['api_timeout'], self.cfg['max_tokens'])
        elif self.cfg['backend'] == 'rex_omni':
            self.backend = RexOmniBackend(self.cfg['rex_model_path'], self.cfg['rex_backend'],
                                          self.cfg['max_tokens'])
        else:
            raise ValueError('backend支持openai或rex_omni')
        self.buffers = {name: deque(maxlen=length) for name, length in
                        {'rgb': 1, 'depth': 30, 'info': 30, 'scan': 1, 'points': 1}.items()}
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = self.snapshot = self.task = None
        self.generation = 0
        self.lock = TargetLock()
        self.last_start = -math.inf
        self.last_result = None
        self.target_valid = False
        self.wait_reason = None
        self.safety_state = {'state': 'unknown', 'scope': 'front_observation_only'}
        for cls, topic, name in [(Image, self.cfg['rgb_topic'], 'rgb'),
                                 (Image, self.cfg['depth_topic'], 'depth'),
                                 (CameraInfo, self.cfg['camera_info_topic'], 'info'),
                                 (LaserScan, self.cfg['scan_topic'], 'scan'),
                                 (PointCloud2, self.cfg['points_topic'], 'points')]:
            self.create_subscription(cls, topic, lambda msg, n=name: self.receive(n, msg), qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/px4_vlm/status', 10)
        self.detections_pub = self.create_publisher(String, '/px4_vlm/detections', 10)
        self.target_pub = self.create_publisher(String, '/px4_vlm/target', 10)
        self.intent_pub = self.create_publisher(String, '/px4_vlm/navigation_request', 10)
        self.safety_pub = self.create_publisher(String, '/px4_vlm/safety', 10)
        self.annotated_pub = self.create_publisher(Image, '/px4_vlm/annotated_image', qos_profile_sensor_data)
        self.point_pub = self.create_publisher(PointStamped, '/px4_vlm/target_point', 10)
        self.world_pub = self.create_publisher(PointStamped, '/px4_vlm/target_point_world', 10)
        self.create_subscription(String, '/px4_vlm/command', self.command, 10)
        self.create_subscription(String, '/px4_vlm/cancel', lambda _: self.cancel(), 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.1, self.safety_tick)
        self.event('ready', '视觉感知节点已就绪；无PX4发布接口，等待图像和目标描述')

    def event(self, event, detail):
        self.status_pub.publish(String(data=json.dumps({'event': event, 'detail': detail}, ensure_ascii=False)))
        self.get_logger().info(f'{event}: {detail}')

    def receive(self, name, msg):
        history = self.buffers[name]
        if name != 'info' and history:
            previous = stamp(history[-1][1])
            if stamp(msg) == previous:
                return  # Replayed frames do not refresh sensor liveness.
            if name == 'rgb' and stamp(msg) < previous:
                self.cancel('图像时间戳回退，取消旧任务并清空缓存')
                for values in self.buffers.values():
                    values.clear()
        history.append((time.monotonic(), msg))

    def latest(self, name):
        history = self.buffers[name]
        if not history or time.monotonic()-history[-1][0] > self.cfg['sensor_timeout']:
            return None
        return history[-1][1]

    def invalidate(self, reason):
        self.target_valid = False
        self.lock.confirmations = 0
        self.target_pub.publish(String(data=json.dumps({'valid': False, 'reason': reason,
            'task_id': self.generation, 'frame_id': None, 'point_optical_m': None})))

    def cancel(self, reason='用户取消任务'):
        self.generation += 1
        self.task = None
        self.lock = TargetLock()
        self.invalidate(reason)
        self.event('cancelled', reason)

    def command(self, msg):
        if msg.data.strip() in ('stop', 'cancel', '停止', '取消'):
            self.cancel(); return
        try:
            task = parse_task(msg.data)
        except (ValueError, TypeError) as exc:
            self.event('rejected', str(exc)); return
        self.generation += 1
        self.task, self.lock, self.last_start = task, TargetLock(), -math.inf
        self.last_result, self.wait_reason = None, None
        self.last_frame_stamp = None
        self.invalidate('新任务，旧目标失效')
        self.event('task', task)

    def capture(self, rgb):
        rgb_time = stamp(rgb)
        depth = info = None
        for name in ('depth', 'info'):
            history = self.buffers[name]
            if history:
                _, candidate = min(history, key=lambda item: abs(stamp(item[1])-rgb_time))
                synchronized = rgb_time > 0 and stamp(candidate) > 0 and abs(stamp(candidate)-rgb_time) <= self.cfg['sync_slop']
                if synchronized or (name == 'info' and stamp(candidate) == 0):
                    if name == 'depth':
                        depth = candidate
                    else:
                        info = candidate
        return {'rgb': rgb, 'depth': depth, 'info': info,
                'received': self.buffers['rgb'][-1][0], 'generation': self.generation,
                'task': dict(self.task)}

    def infer(self, snapshot):
        image = rgb_image(snapshot['rgb'])
        resized = image.copy()
        resized.thumbnail((self.cfg['max_image_side'], self.cfg['max_image_side']))
        result = self.backend.infer(resized, snapshot['task']['target'])
        return image, result

    def tick(self):
        now = time.monotonic()
        if self.last_result is not None and now-self.last_result > self.cfg['max_result_age'] and self.target_valid:
            self.invalidate('目标观测过期')
        if self.future is not None and self.future.done():
            future, snapshot = self.future, self.snapshot
            self.future = self.snapshot = None
            if snapshot['generation'] == self.generation:
                try:
                    image, result = future.result()
                    if now-snapshot['received'] > self.cfg['max_result_age']:
                        self.invalidate('模型结果太旧，不能作为当前目标')
                        self.event('stale_result', '请降低分辨率、加速推理或调整max_result_age；不输出目标点')
                    else:
                        self.process_result(snapshot, image, result)
                except Exception as exc:
                    self.invalidate('模型请求或结果处理失败')
                    self.event('error', f'{type(exc).__name__}: {exc}')
                if self.task is not None and self.task['mode'] == 'observe':
                    self.task = None
        if self.future is not None or self.task is None or now-self.last_start < self.cfg['inference_period']:
            return
        rgb = self.latest('rgb')
        if rgb is None:
            if self.wait_reason != 'rgb':
                self.wait_reason = 'rgb'
                self.event('waiting_sensor', f"等待新鲜RGB图像: {self.cfg['rgb_topic']}")
            return
        if stamp(rgb) == getattr(self, 'last_frame_stamp', None):
            return
        self.wait_reason = None
        self.snapshot = self.capture(rgb)
        self.last_start, self.last_frame_stamp = now, stamp(rgb)
        self.future = self.pool.submit(self.infer, self.snapshot)

    def point_for(self, snapshot, box):
        rgb, depth, info = snapshot['rgb'], snapshot['depth'], snapshot['info']
        if not self.cfg['depth_registered'] or not self.cfg['images_rectified']:
            raise ValueError('需启用并实际提供已配准且校正的RGBD，当前只输出二维框')
        if depth is None or info is None:
            raise ValueError('缺少同步深度/CameraInfo')
        frame = self.cfg['optical_frame']
        if not frame or not all(m.header.frame_id == frame for m in (rgb, depth, info)):
            raise ValueError('RGB/配准深度/内参必须对应已验证的同一optical_frame')
        if (rgb.width, rgb.height) != (depth.width, depth.height) or (info.width, info.height) != (rgb.width, rgb.height):
            raise ValueError('RGBD和内参分辨率不匹配，不按比例猜测配准')
        # Rectified images use projection matrix P, not raw distorted-image K.
        p = info.p
        if any(abs(p[i]) > 1e-8 for i in (1, 3, 4, 7)):
            raise ValueError('当前反投影要求零skew/零投影平移的校正RGB相机内参')
        k = [p[0], p[1], p[2], p[4], p[5], p[6], 0, 0, 1]
        xyz = project_box(box, depth_array(depth), k)
        point = PointStamped()
        point.header = rgb.header
        point.point.x, point.point.y, point.point.z = xyz
        return point

    def process_result(self, snapshot, image, result):
        self.last_result = snapshot['received']
        index, association = self.lock.select(result['detections'])
        metadata = {'task_id': self.generation, 'image_stamp': stamp(snapshot['rgb']),
                    'frame_id': snapshot['rgb'].header.frame_id,
                    'width': image.width, 'height': image.height,
                    'observation_age_s': time.monotonic()-snapshot['received'],
                    'max_age_s': self.cfg['max_result_age']}
        self.detections_pub.publish(String(data=json.dumps({**metadata, **result,
            'selected_index': index, 'association': association}, ensure_ascii=False)))
        draw = ImageDraw.Draw(image)
        for i, detection in enumerate(result['detections']):
            box = detection['bbox']
            coords = [box[0]*image.width, box[1]*image.height, box[2]*image.width, box[3]*image.height]
            draw.rectangle(coords, outline='lime' if i == index else 'yellow', width=3)
            draw.text((coords[0], coords[1]), str(i), fill='lime')
        annotated = Image()
        annotated.header = snapshot['rgb'].header
        annotated.width, annotated.height, annotated.encoding = image.width, image.height, 'rgb8'
        annotated.step, annotated.data = image.width*3, image.tobytes()
        self.annotated_pub.publish(annotated)
        if index is None:
            self.invalidate(association)
            self.event(association, '未锁定唯一目标；请使用更明确的颜色/位置/类别描述')
            return
        detection = result['detections'][index]
        target = {**metadata, 'valid': True, 'track_id': f'{self.generation}:1',
                  'label': detection['label'], 'bbox': detection['bbox'],
                  'confirmations': self.lock.confirmations, 'point_optical_m': None,
                  'point_world_m': None, 'world_frame': None, 'depth_reason': None}
        point = None
        try:
            point = self.point_for(snapshot, detection['bbox'])
            target['point_optical_m'] = [point.point.x, point.point.y, point.point.z]
            self.point_pub.publish(point)
        except ValueError as exc:
            target['depth_reason'] = str(exc)
        if point is not None and self.cfg['world_frame']:
            try:
                transform = self.tf_buffer.lookup_transform(self.cfg['world_frame'], point.header.frame_id,
                                                            Time.from_msg(point.header.stamp))
                world = do_transform_point(point, transform)
                target['point_world_m'] = [world.point.x, world.point.y, world.point.z]
                target['world_frame'] = world.header.frame_id
                self.world_pub.publish(world)
            except TransformException:
                target['world_transform_reason'] = '缺少图像时刻的光学相机到世界坐标TF'
        self.target_valid = True
        self.target_pub.publish(String(data=json.dumps(target, ensure_ascii=False)))
        if snapshot['task']['mode'] in ('track', 'navigate'):
            intent = {**target, 'mode': snapshot['task']['mode'],
                      'standoff_m': snapshot['task']['standoff_m'],
                      'stable': self.lock.confirmations >= 2,
                      'obstacle_observation': self.safety_state,
                      'executable': False,
                      'requires': ['calibrated_world_transform', 'local_3d_planner', 'flight_executor_adapter']}
            self.intent_pub.publish(String(data=json.dumps(intent, ensure_ascii=False)))
        self.event('target', {'association': association, 'has_depth': point is not None,
                              'confirmations': self.lock.confirmations})

    def safety_tick(self):
        distances, reasons = {}, {}
        for source in ('depth', 'scan', 'points'):
            msg = self.latest(source)
            value = None
            try:
                if msg is None:
                    raise ValueError('缺失或过期')
                if source == 'depth':
                    if not self.cfg['depth_forward_facing']:
                        raise ValueError('尚未确认深度相机朝前')
                    value = depth_front_distance(depth_array(msg))
                elif source == 'scan':
                    if msg.header.frame_id != self.cfg['scan_frame']:
                        raise ValueError('扫描坐标系不匹配')
                    angles = msg.angle_min + np.arange(len(msg.ranges))*msg.angle_increment
                    difference = np.arctan2(np.sin(angles-math.radians(self.cfg['scan_forward_angle_deg'])),
                                            np.cos(angles-math.radians(self.cfg['scan_forward_angle_deg'])))
                    ranges = np.asarray(msg.ranges)
                    values = ranges[np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
                                    & (abs(difference) <= math.radians(self.cfg['scan_half_angle_deg']))]
                    value = float(np.min(values)) if values.size else None
                else:
                    if msg.header.frame_id != self.cfg['points_frame']:
                        raise ValueError('点云需先变换到配置的前向x坐标系')
                    value = cloud_front_distance(msg)
                if value is None:
                    raise ValueError('无足够有效回波，不能据此认定无障碍')
            except (ValueError, TypeError, IndexError) as exc:
                reasons[source] = str(exc)
            distances[source] = value
        required = self.cfg['required_obstacle_sources']
        nearest = min((d for d in distances.values() if d is not None), default=None)
        if nearest is not None and nearest < self.cfg['stop_distance']:
            state = 'blocked'
        elif required and all(distances[source] is not None for source in required):
            state = 'clear_observed'
        else:
            state = 'unknown'
        self.safety_state = {'state': state, 'scope': 'front_observation_only',
            'distances_m': distances, 'reasons': reasons, 'stop_distance_m': self.cfg['stop_distance'],
            'stamp': self.get_clock().now().nanoseconds/1e9, 'max_age_s': self.cfg['sensor_timeout'],
            'collision_free_path_verified': False}
        self.safety_pub.publish(String(data=json.dumps(self.safety_state, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MultimodalModelControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.pool.shutdown(wait=False, cancel_futures=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
