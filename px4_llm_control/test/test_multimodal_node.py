from collections import deque
from concurrent.futures import Future
from types import SimpleNamespace as NS
import json
import numpy as np
import pytest

pytest.importorskip('rclpy')
from sensor_msgs.msg import Image, CameraInfo
from PIL import Image as PILImage
from px4_llm_control.multimodal_core import TargetLock
from px4_llm_control.multimodal_model_control import MultimodalModelControl


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def fake_node(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr('px4_llm_control.multimodal_model_control.time.monotonic', lambda: clock[0])
    n = object.__new__(MultimodalModelControl)
    n.cfg = dict(depth_registered=True, images_rectified=True, optical_frame='camera_optical',
        world_frame='', sensor_timeout=1., max_result_age=3., inference_period=1., sync_slop=.1,
        required_obstacle_sources=['depth'], depth_forward_facing=True, scan_frame='laser',
        points_frame='base_link', stop_distance=2.)
    n.generation = 1
    n.task = dict(mode='track', target='red box', standoff_m=3)
    n.lock = TargetLock()
    n.target_valid, n.last_result = False, None
    n.future = n.snapshot = None
    n.buffers = {name: deque(maxlen=30) for name in ('rgb', 'depth', 'info', 'scan', 'points')}
    n.events = []
    n.event = lambda event, detail: n.events.append((event, detail))
    n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
    for name in ('detections', 'target', 'intent', 'annotated', 'point', 'world', 'safety'):
        setattr(n, name+'_pub', Publisher())
    n.safety_state = dict(state='unknown')
    return n, clock


def snapshot():
    rgb = Image(width=20, height=20, encoding='rgb8', step=60, data=bytes([255, 0, 0])*400)
    rgb.header.frame_id, rgb.header.stamp.sec = 'camera_optical', 10
    depth = Image(width=20, height=20, encoding='32FC1', step=80,
                  data=np.full((20, 20), 4, dtype='<f4').tobytes())
    depth.header = rgb.header
    info = CameraInfo(width=20, height=20)
    info.header = rgb.header
    info.p = [20., 0., 10., 0., 0., 20., 10., 0., 0., 0., 1., 0.]
    return dict(rgb=rgb, depth=depth, info=info, received=10., generation=1,
                task=dict(mode='track', target='red box', standoff_m=3))


def test_rgbd_result_becomes_optical_point_but_never_flight_command(monkeypatch):
    n, _ = fake_node(monkeypatch)
    result = dict(description='box', detections=[dict(label='red box', bbox=[.2, .2, .8, .8])])
    n.process_result(snapshot(), PILImage.new('RGB', (20, 20)), result)
    target = json.loads(n.target_pub.messages[-1].data)
    assert target['point_optical_m'] == [0, 0, 4]
    intent = json.loads(n.intent_pub.messages[-1].data)
    assert intent['executable'] is False
    assert intent['stable'] is False
    n.process_result(snapshot(), PILImage.new('RGB', (20, 20)), result)
    assert json.loads(n.intent_pub.messages[-1].data)['stable'] is True
    assert not hasattr(n, 'command_pub')


@pytest.mark.parametrize('change', ['unregistered', 'wrong_frame', 'wrong_size', 'no_depth', 'raw_images'])
def test_projection_requires_real_registration_and_calibration(monkeypatch, change):
    n, _ = fake_node(monkeypatch)
    snap = snapshot()
    if change == 'unregistered':
        n.cfg['depth_registered'] = False
    elif change == 'wrong_frame':
        n.cfg['optical_frame'] = 'body'
    elif change == 'wrong_size':
        snap['depth'].width = 10
    elif change == 'no_depth':
        snap['depth'] = None
    elif change == 'raw_images':
        n.cfg['images_rectified'] = False
    with pytest.raises(ValueError):
        n.point_for(snap, [.2, .2, .8, .8])


def test_missing_obstacle_sensors_are_unknown(monkeypatch):
    n, _ = fake_node(monkeypatch)
    n.safety_tick()
    assert n.safety_state['state'] == 'unknown'
    assert n.safety_state['collision_free_path_verified'] is False


def test_cancel_discards_late_inference_result(monkeypatch):
    n, _ = fake_node(monkeypatch)
    n.future = Future()
    n.snapshot = snapshot()
    n.future.set_result((PILImage.new('RGB', (20, 20)), {'invalid': 'old response'}))
    n.cancel()
    n.tick()
    assert not n.target_valid
    assert not n.detections_pub.messages


def test_depth_pairing_uses_header_stamps_not_arrival(monkeypatch):
    n, _ = fake_node(monkeypatch)
    snap = snapshot()
    n.buffers['rgb'].append((10., snap['rgb']))
    old_depth = Image()
    old_depth.header.stamp.sec = 2
    n.buffers['depth'].append((10., old_depth))
    assert n.capture(snap['rgb'])['depth'] is None
