"""ROS-independent image, depth and target association utilities."""
import json
import math
import re

import numpy as np
from PIL import Image


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def parse_task(text):
    text = text.strip()
    if not text:
        raise ValueError('任务不能为空')
    if text.startswith('{'):
        task = json.loads(text)
        if not isinstance(task, dict) or set(task) - {'mode', 'target', 'standoff_m'}:
            raise ValueError('任务字段仅支持mode/target/standoff_m')
    else:
        mode, target = 'observe', text
        for prefix, value in [('跟踪', 'track'), ('导航到', 'navigate'), ('观察', 'observe'), ('寻找', 'observe')]:
            if text.startswith(prefix):
                mode, target = value, text[len(prefix):].strip(' ：:')
                break
        task = {'mode': mode, 'target': target}
    task = {'mode': 'observe', 'standoff_m': 3.0, **task}
    if task['mode'] not in ('observe', 'track', 'navigate'):
        raise ValueError('mode必须为observe/track/navigate')
    if not isinstance(task.get('target'), str) or not task['target'].strip():
        raise ValueError('target必须为非空自然语言目标描述')
    if not finite(task['standoff_m']) or not 1 <= task['standoff_m'] <= 20:
        raise ValueError('standoff_m范围1到20米')
    return task


def image_array(msg):
    """Respect ROS row padding and endian. Avoid cv_bridge/numpy ABI coupling."""
    layouts = {'rgb8': ('u1', 3), 'bgr8': ('u1', 3), 'rgba8': ('u1', 4),
               'bgra8': ('u1', 4), 'mono8': ('u1', 1),
               '16UC1': ('u2', 1), '32FC1': ('f4', 1)}
    if msg.encoding not in layouts or msg.height <= 0 or msg.width <= 0:
        raise ValueError('不支持的Image编码或尺寸')
    dtype, channels = layouts[msg.encoding]
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + dtype)
    pixel = dtype.itemsize*channels
    if msg.step < msg.width*pixel or len(msg.data) < msg.step*msg.height:
        raise ValueError('Image数据长度或step无效')
    shape = (msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width)
    strides = (msg.step, pixel, dtype.itemsize) if channels > 1 else (msg.step, pixel)
    return np.ndarray(shape, dtype=dtype, buffer=bytes(msg.data), strides=strides)


def rgb_image(msg):
    array = image_array(msg)
    if msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'):
        raise ValueError('RGB输入不是彩色或mono8图像')
    if msg.encoding in ('bgr8', 'bgra8'):
        array = array[..., [2, 1, 0]]
    return Image.fromarray(array).convert('RGB')


def depth_array(msg):
    if msg.encoding not in ('16UC1', '32FC1'):
        raise ValueError('深度需为16UC1毫米或32FC1米')
    array = image_array(msg).astype(np.float32)
    return array*0.001 if msg.encoding == '16UC1' else array


def parse_detections(text):
    # Accept a single fenced JSON block, but never extract arbitrary substrings/code.
    text = text.strip()
    if text.startswith('```'):
        text = re.sub(r'^```(?:json)?\s*', '', text)
        text = re.sub(r'\s*```$', '', text)
    data = json.loads(text)
    if (not isinstance(data, dict) or set(data) != {'description', 'detections'}
            or not isinstance(data['description'], str) or not isinstance(data['detections'], list)):
        raise ValueError('VLM输出需为description/detections')
    for detection in data['detections']:
        if (not isinstance(detection, dict) or set(detection) != {'label', 'bbox'}
                or not isinstance(detection['label'], str) or not detection['label'].strip()):
            raise ValueError('检测字段需为label/bbox')
        box = detection['bbox']
        if (not isinstance(box, list) or len(box) != 4 or not all(finite(v) for v in box)
                or not 0 <= box[0] < box[2] <= 1 or not 0 <= box[1] < box[3] <= 1):
            raise ValueError('bbox必须为归一化[0,1]的有效xyxy；不猜测像素/1000坐标单位')
    return data


def iou(a, b):
    intersection = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
    area = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/area if area else 0.0


class TargetLock:
    """Conservative IoU association, not a re-identification or motion tracker."""
    def __init__(self, min_iou=0.2):
        self.box = None
        self.min_iou = min_iou
        self.confirmations = 0

    def select(self, detections):
        if not detections:
            self.confirmations = 0
            return None, 'lost'
        if self.box is None:
            if len(detections) != 1:
                return None, 'ambiguous'
            index = 0
        else:
            scores = sorted(((iou(self.box, d['bbox']), i) for i, d in enumerate(detections)), reverse=True)
            if scores[0][0] < self.min_iou:
                self.confirmations = 0
                return None, 'lost'
            if len(scores) > 1 and scores[0][0]-scores[1][0] < 0.1:
                self.confirmations = 0
                return None, 'ambiguous'
            index = scores[0][1]
        self.box = list(detections[index]['bbox'])
        self.confirmations += 1
        return index, 'locked'


def project_box(box, depth, k):
    """Aligned, rectified optical image only. Returns visible surface estimate."""
    height, width = depth.shape
    u = (box[0]+box[2])*0.5*width
    v = (box[1]+box[3])*0.5*height
    radius = max(1, int(min((box[2]-box[0])*width, (box[3]-box[1])*height)*0.15))
    x, y = min(width-1, int(u)), min(height-1, int(v))
    patch = depth[max(0, y-radius):min(height, y+radius+1), max(0, x-radius):min(width, x+radius+1)]
    values = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 100)]
    if values.size < 3 or values.size < patch.size*0.25:
        raise ValueError('目标区域有效深度不足')
    if len(k) != 9 or not all(math.isfinite(float(value)) for value in k) or k[0] <= 0 or k[4] <= 0:
        raise ValueError('相机内参无效')
    z = float(np.median(values))
    return [(u-k[2])*z/k[0], (v-k[5])*z/k[4], z]


def depth_front_distance(depth):
    height, width = depth.shape
    patch = depth[height//3:2*height//3, width//3:2*width//3]
    valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 100)]
    # A diagnostic in a central image region, not a swept-volume collision test.
    return float(np.min(valid)) if valid.size >= max(3, patch.size*0.25) else None


def cloud_front_distance(msg, half_width=0.75, half_height=0.75):
    """XYZ cloud in a calibrated forward-x frame, including row padding."""
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields for name in ('x', 'y', 'z')):
        raise ValueError('点云必须包含xyz')
    axes = []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.datatype not in (7, 8) or field.count != 1:
            raise ValueError('xyz字段需为FLOAT32或FLOAT64')
        dtype = np.dtype(('>' if msg.is_bigendian else '<') + ('f4' if field.datatype == 7 else 'f8'))
        if field.offset+dtype.itemsize > msg.point_step:
            raise ValueError('点云字段偏移无效')
        if msg.row_step < msg.width*msg.point_step or len(msg.data) < msg.row_step*msg.height:
            raise ValueError('点云长度无效')
        axes.append(np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                               offset=field.offset, strides=(msg.row_step, msg.point_step)).ravel())
    x, y, z = axes
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (x > 0.1) & (abs(y) <= half_width) & (abs(z) <= half_height)
    return float(np.min(x[valid])) if np.any(valid) else None
