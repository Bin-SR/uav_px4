import math
import pytest
from px4_llm_control.planner import validate_plan
from px4_llm_control.trajectory import Circuit, Line, Profile, Waypoints, bearing, wrap


def square(**kwargs):
    return dict(action='square', side=6, height=2, speed=3) | kwargs


def test_more_than_sixteen_actions():
    actions = [dict(action='hover', seconds=1)] * 1000
    assert len(validate_plan(dict(explanation='', actions=actions))) == 1000


def test_six_metre_square_centered_at_takeoff():
    route = Circuit(square(), [10, 20, -2], [10, 20, 0])
    assert route.end == pytest.approx([13, 23, -2])
    assert len(route.edges) == 4
    for edge in route.edges:
        assert math.dist(edge.start, edge.end) == pytest.approx(6)
        assert abs(edge.start[0]-10) == pytest.approx(3)
        assert abs(edge.start[1]-20) == pytest.approx(3)
        _, v, _ = edge.sample(edge.duration/2)
        assert math.hypot(*v) == pytest.approx(3)
        assert edge.sample(edge.duration)[1] == [0, 0, 0]
    assert route.sample(route.duration)[0] == pytest.approx(route.end)


def test_reference_continuity_and_bounds():
    route = Circuit(square(laps=3), [0, 0, -2], [0, 0, 0])
    previous = route.sample(0)[0]
    for i in range(1, math.ceil(route.duration*100)+1):
        p, v, a = route.sample(i/100)
        assert math.dist(previous, p) <= 0.03001
        assert math.hypot(*v) <= 3.00001
        assert math.hypot(*a) <= 2.00001
        assert p[2] == -2
        previous = p


def test_laps_do_not_expand_memory():
    route = Circuit(square(laps=1000000), [0, 0, -2], [0, 0, 0])
    assert len(route.edges) == 4
    assert route.duration > 180
    assert route.sample(route.duration)[0] == pytest.approx(route.end)


def test_circle_position_velocity_and_acceleration():
    step = dict(action='circle', radius=6, height=2, speed=3, laps=2)
    route = Circuit(step, [0, 0, -2], [0, 0, 0])
    for i in range(1001):
        t = route.approach.duration + route.body_duration*i/1000
        p, v, a = route.sample(t)
        assert math.hypot(*p[:2]) == pytest.approx(6)
        assert math.hypot(*v) <= 3.00001
        assert math.hypot(*a) <= 2.00001
    t = route.approach.duration + route.body_duration/3
    p0 = route.sample(t-0.0001)[0]
    p1, v1, _ = route.sample(t)
    p2 = route.sample(t+0.0001)[0]
    assert [(b-a)/0.0002 for a, b in zip(p0, p2)] == pytest.approx(v1, abs=1e-5)


def test_circle_acceleration_limits_requested_speed():
    route = Circuit(dict(action='circle', radius=0.5, height=2, speed=5),
                    [0, 0, -2], [0, 0, 0])
    assert route.speed < 5


@pytest.mark.parametrize('step', [square(center=[19, 0]),
    dict(action='circle', radius=6, height=2, speed=3, center=[15, 0]),
    dict(action='path', points=[[0, 0, 11]], speed=1)])
def test_reject_entire_out_of_bounds_route(step):
    with pytest.raises(ValueError):
        cls = Waypoints if step['action'] == 'path' else Circuit
        cls(step, [0, 0, -2], [0, 0, 0])


@pytest.mark.parametrize('step', [square(laps=0), square(laps=True), square(speed=6),
    square(center=[float('nan'), 0]), square(heading='pitch'),
    dict(action='path', points=[], speed=2)])
def test_invalid_shapes(step):
    with pytest.raises(ValueError):
        validate_plan(dict(explanation='', actions=[step]))


def test_strict_pitch_rejected_and_best_effort_is_explicit():
    plan = dict(explanation='角度由PX4控制', actions=[square()],
                attitude=dict(pitch_deg=0, strict=True))
    with pytest.raises(ValueError, match='俯仰'):
        validate_plan(plan)
    plan['attitude']['strict'] = False
    assert validate_plan(plan)


def test_profile_endpoints_and_short_leg():
    profile = Profile(0.1, 3, 2)
    assert profile.speed < 3
    assert profile.sample(profile.duration) == (0.1, 0, 0)
    assert Line([1, 2, 3], [1, 2, 3]).sample(0)[0] == [1, 2, 3]


def test_center_heading_and_wrap():
    assert bearing([3, 3, -2], [0, 0], 0) == pytest.approx(-3*math.pi/4)
    assert bearing([0, 0, -2], [0, 0], 1) == 1
    assert wrap(math.radians(359)) == pytest.approx(math.radians(-1))


def test_waypoints_absolute_height():
    route = Waypoints(dict(action='path', points=[[3, 3, 2], [-3, 3, 5]], speed=2),
                      [10, 20, 0], [10, 20, 0])
    assert route.end == [7, 23, -5]
