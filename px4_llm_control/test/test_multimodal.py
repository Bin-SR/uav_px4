import io
import json
import struct
from types import SimpleNamespace as NS

import numpy as np
from PIL import Image
import pytest

from px4_llm_control.multimodal_core import (parse_task, parse_detections, rgb_image,
    depth_array, project_box, TargetLock, depth_front_distance, cloud_front_distance)
from px4_llm_control.vlm_backends import OpenAIVisionBackend, RexOmniBackend


def image_msg(data, encoding='rgb8', width=2, height=2, step=8, endian=False):
    return NS(data=data, encoding=encoding, width=width, height=height, step=step, is_bigendian=endian)


def test_natural_task_and_json_task():
    assert parse_task('跟踪红色汽车')['mode'] == 'track'
    assert parse_task('导航到蓝色箱子')['target'] == '蓝色箱子'
    assert parse_task('红色球')['mode'] == 'observe'
    assert parse_task('{"target":"box","mode":"track","standoff_m":4}')['standoff_m'] == 4


@pytest.mark.parametrize('task', ['', '{}', '{"target":"box","mode":"fly"}',
    '{"target":"box","standoff_m":true}', '{"target":"box","shell":"x"}'])
def test_invalid_task(task):
    with pytest.raises(ValueError):
        parse_task(task)


def test_rgb_padding_and_bgr_conversion():
    raw = bytes([0, 0, 255, 0, 255, 0, 99, 99, 255, 0, 0, 255, 255, 255, 99, 99])
    image = rgb_image(image_msg(raw, 'bgr8'))
    assert image.getpixel((0, 0)) == (255, 0, 0)
    assert image.getpixel((0, 1)) == (0, 0, 255)


def test_depth_units_endian_and_padding():
    msg = image_msg(struct.pack('>HHxxHHxx', 2000, 3000, 0, 4000), '16UC1', step=6, endian=True)
    assert depth_array(msg) == pytest.approx(np.array([[2, 3], [0, 4]]))


def test_short_buffer_rejected():
    with pytest.raises(ValueError):
        rgb_image(image_msg(b'bad'))


def test_projection_uses_measurement_not_model_distance():
    depth = np.full((100, 100), 4, dtype=np.float32)
    point = project_box([0.4, 0.4, 0.6, 0.6], depth, [100, 0, 50, 0, 100, 50, 0, 0, 1])
    assert point == [0, 0, 4]
    depth[:] = np.nan
    with pytest.raises(ValueError):
        project_box([0.4, 0.4, 0.6, 0.6], depth, [100, 0, 50, 0, 100, 50, 0, 0, 1])


@pytest.mark.parametrize('bbox', [[0, 0, 1000, 1000], [0.5, 0, 0.2, 1],
    [0, 0, float('nan'), 1], [False, 0, 1, 1]])
def test_never_guess_box_coordinate_scale(bbox):
    with pytest.raises(ValueError):
        parse_detections(json.dumps(dict(description='', detections=[dict(label='box', bbox=bbox)])))


def test_no_random_selection_or_automatic_identity_switch():
    lock = TargetLock()
    a, b = dict(label='box', bbox=[0.1, 0.1, 0.3, 0.3]), dict(label='box', bbox=[0.7, 0.7, 0.9, 0.9])
    assert lock.select([a, b]) == (None, 'ambiguous')
    assert lock.select([a]) == (0, 'locked')
    assert lock.select([b]) == (None, 'lost')
    assert lock.select([a, b]) == (0, 'locked')
    assert lock.confirmations == 1


def test_no_depth_returns_unknown_not_free():
    assert depth_front_distance(np.full((20, 20), np.inf)) is None
    assert depth_front_distance(np.full((20, 20), 1.2)) == pytest.approx(1.2)


def test_pointcloud_front_corridor_and_row_padding():
    fields = [NS(name=name, datatype=7, count=1, offset=i*4) for i, name in enumerate('xyz')]
    msg = NS(fields=fields, is_bigendian=False, height=2, width=1, point_step=12, row_step=16,
             data=struct.pack('<fffxxxxfffxxxx', 5, 0, 0, 1.5, 0, 0))
    assert cloud_front_distance(msg) == 1.5


def test_qwen_api_sends_image_and_normalized_json(monkeypatch):
    result = dict(description='box', detections=[dict(label='red box', bbox=[0.1, 0.2, 0.5, 0.8])])

    def fake_open(request, timeout):
        payload = json.loads(request.data)
        assert payload['messages'][1]['content'][0]['image_url']['url'].startswith('data:image/jpeg;base64,')
        assert payload['model'] == 'qwen'
        return io.BytesIO(json.dumps({'choices': [{'finish_reason': 'stop',
            'message': {'content': json.dumps(result)}}]}).encode())

    monkeypatch.setattr('urllib.request.urlopen', fake_open)
    assert OpenAIVisionBackend('http://localhost/v1', 'qwen').infer(Image.new('RGB', (20, 20)), '红箱子') == result


def test_rex_official_prediction_adapter_without_model_download():
    backend = RexOmniBackend('/not-loaded')
    backend.wrapper = NS(inference=lambda **_: [{'extracted_predictions': {'box': [
        {'type': 'box', 'coords': [10, 20, 50, 80]}]}}])
    result = backend.infer(Image.new('RGB', (100, 100)), 'box')
    assert result['detections'][0]['bbox'] == [0.1, 0.2, 0.5, 0.8]
