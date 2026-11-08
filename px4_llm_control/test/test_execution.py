"""Exercise the real flight state machine with synthetic telemetry, no DDS or motors."""
import math
from collections import deque
from types import SimpleNamespace
import pytest

pytest.importorskip('rclpy')
from px4_msgs.msg import VehicleStatus
from px4_llm_control.node import LLMControl


def make_node(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr('px4_llm_control.node.time.monotonic', lambda: clock[0])
    n = object.__new__(LLMControl)
    n.acceleration, n.yaw_rate = 2.0, math.pi/2
    n.position = SimpleNamespace(x=0., y=0., z=-2., heading=0., vx=0., vy=0., vz=0.)
    n.status = SimpleNamespace(failsafe=False, arming_state=VehicleStatus.ARMING_STATE_ARMED,
                               nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD)
    n.origin, n.target, n.yaw = [0., 0., 0.], [0., 0., -2.], 0.
    n.future, n.step, n.motion = None, None, None
    n.actions = deque()
    n.phase, n.owned = 'hold', True
    n.generation = n.started = n.last_tick = 0
    n.valid = lambda: True
    n.reset_counts = lambda: (0, 0, 0)
    n.resets = (0, 0, 0)
    n.events_seen = []
    n.event = lambda event, detail: n.events_seen.append((event, detail))
    n.streams = []
    n.stream = lambda: n.streams.append((list(n.target), list(n.velocity), n.yaw))
    n.clear_motion()
    return n, clock


def follow_reference(n):
    n.position.x, n.position.y, n.position.z = n.target
    n.position.heading = n.yaw
    n.position.vx, n.position.vy, n.position.vz = n.velocity


def test_square_runs_with_continuous_yaw_and_finishes(monkeypatch):
    n, clock = make_node(monkeypatch)
    n.actions = deque([dict(action='square', side=6, height=2, speed=3, heading='center')])
    n.next_step()
    headings = []
    for i in range(3000):
        clock[0] += 0.05
        follow_reference(n)
        n.tick()
        headings.append(n.yaw)
        if n.phase == 'hold':
            break
    assert n.phase == 'hold'
    assert any(event == 'completed' for event, _ in n.events_seen)
    assert math.dist(n.target, [3, 3, -2]) < 1e-6
    assert len(set(round(yaw, 2) for yaw in headings)) > 50
    assert any(math.hypot(*v) > 2.9 for _, v, _ in n.streams)
    assert all(abs(math.atan2(math.sin(b-a), math.cos(b-a))) <= n.yaw_rate*0.05+1e-6
               for a, b in zip(headings, headings[1:]))


def test_stop_clears_velocity_and_late_motion(monkeypatch):
    n, clock = make_node(monkeypatch)
    n.actions = deque([dict(action='move', frame='ned', dx=6, dy=0, dz=0, speed=3)])
    n.next_step()
    clock[0] = 0.05
    n.tick()
    assert n.velocity[0] > 0
    n.emergency(SimpleNamespace(data='stop'))
    assert n.phase == 'hold'
    assert n.motion is None
    assert n.velocity == [0, 0, 0]
    assert not n.actions


def test_large_tracking_error_lands(monkeypatch):
    n, clock = make_node(monkeypatch)
    n.actions = deque([dict(action='square', side=6, height=2, speed=3)])
    n.next_step()
    n.position.x = 10
    clock[0] += 0.05
    n.tick()
    assert n.phase == 'landing'
    assert n.motion is None


def test_long_hover_not_limited_to_180_seconds(monkeypatch):
    n, clock = make_node(monkeypatch)
    n.actions = deque([dict(action='hover', seconds=600)])
    n.next_step()
    for t in (1, 181, 500, 602):
        clock[0] = t
        n.tick()
    assert n.phase == 'hold'


def test_all_route_prevalidated_before_takeoff(monkeypatch):
    n, _ = make_node(monkeypatch)
    n.owned, n.phase = False, 'idle'
    n.status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
    n.position.z = 0
    n.land = SimpleNamespace(landed=True)
    n.plans = SimpleNamespace(publish=lambda _: None)
    plan = dict(explanation='bad late path', actions=[dict(action='takeoff', height=2),
                dict(action='square', side=6, height=2, speed=3, center=[19, 0])])
    with pytest.raises(ValueError, match='路径超出'):
        n.accept(plan)
    assert n.phase == 'idle'
    assert not n.actions


def test_emergency_land_invalidates_inflight_plan_and_velocity(monkeypatch):
    n, _ = make_node(monkeypatch)
    n.velocity = [3, 0, 0]
    generation = n.generation
    n.emergency(SimpleNamespace(data='land'))
    assert n.generation > generation
    assert n.phase == 'landing'
    assert n.velocity == [0, 0, 0]
