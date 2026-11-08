import math
import pytest
from px4_llm_control.planner import validate_plan, target_for


def test_body_rotation_and_up():
    point, _ = target_for(dict(action='move', frame='body', dx=2, dy=0, dz=-1),
                          [0, 0, -2], math.pi / 2, [0, 0, 0])
    assert point == pytest.approx([0, 2, -3])


@pytest.mark.parametrize('step', [
    dict(action='move', frame='body', dx=float('nan'), dy=0, dz=0),
    dict(action='takeoff', height=True), dict(action='takeoff', height=11),
    dict(action='shell', command='anything'), dict(action='land', extra=1),
    dict(action='hover', seconds=-1), dict(action='yaw', degrees=181)])
def test_reject_invalid(step):
    with pytest.raises(ValueError):
        validate_plan(dict(explanation='test', actions=[step]))


def test_radius_is_cumulative():
    with pytest.raises(ValueError):
        target_for(dict(action='move', frame='ned', dx=10, dy=0, dz=0),
                   [15, 0, -2], 0, [0, 0, 0])


def test_landing_must_be_last():
    with pytest.raises(ValueError):
        validate_plan(dict(explanation='', actions=[dict(action='land'),
                      dict(action='takeoff', height=2)]))


def test_api_json_request(monkeypatch):
    import io
    import json
    from px4_llm_control.planner import request_plan
    plan = dict(explanation='起飞', actions=[dict(action='takeoff', height=2)])

    def fake_open(req, timeout):
        payload = json.loads(req.data)
        assert payload['response_format'] == {'type': 'json_object'}
        assert timeout == 30
        assert req.get_header('Authorization') == 'Bearer test-key'
        return io.BytesIO(json.dumps({'choices': [{'finish_reason': 'stop',
                         'message': {'content': json.dumps(plan)}}]}).encode())

    monkeypatch.setattr('urllib.request.urlopen', fake_open)
    assert request_plan('起飞', {}, 'test-key', 'deepseek-chat',
                        'https://api.deepseek.com', 30) == plan
