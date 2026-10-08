from commander_operator.commander_operator_node import Route, Waypoint
from crl_commander_interfaces.msg import OperatorGoalArray
import pytest

WGS84_A = 6378137.0  # equatorial radius [m]
WGS84_B = 6356752.314245  # polar radius [m]


def test_local_waypoint():
    w = Waypoint('dock')
    success, _ = w.from_dict({'x': 3.2, 'y': -1.0}, 'map')

    assert success
    assert w.name == 'dock'
    assert w.goal.goal.header.frame_id == 'map'
    assert w.goal.goal.pose.position.x == pytest.approx(3.2)
    assert w.goal.goal.pose.position.y == pytest.approx(-1.0)
    assert w.goal.goal.pose.position.z == 0.0


def test_optional_fields_default_to_unset():
    w = Waypoint()
    w.from_dict({'x': 0.0, 'y': 0.0}, 'map')

    assert not w.goal.use_max_linear_vel
    assert not w.goal.use_max_angular_vel
    assert not w.goal.use_lookahead_distance
    assert w.goal.mandatory


def test_optional_fields_are_set():
    w = Waypoint()
    w.from_dict({'x': 0.0, 'y': 0.0, 'max_linear_vel': 1.2, 'max_angular_vel': 0.8,
                 'lookahead_distance': 1.5, 'mandatory': False}, 'map')

    assert w.goal.use_max_linear_vel
    assert w.goal.max_linear_vel == pytest.approx(1.2)
    assert w.goal.use_max_angular_vel
    assert w.goal.max_angular_vel == pytest.approx(0.8)
    assert w.goal.use_lookahead_distance
    assert w.goal.lookahead_distance == pytest.approx(1.5)
    assert not w.goal.mandatory


def test_integer_values_become_floats():
    # YAML 'x: 1' is an int; float64 message fields must get a float.
    w = Waypoint()
    w.from_dict({'x': 1, 'y': 2, 'max_linear_vel': 1, 'max_angular_vel': 1,
                 'lookahead_distance': 2}, 'map')

    for value in (w.goal.goal.pose.position.x, w.goal.goal.pose.position.y,
                  w.goal.goal.pose.position.z, w.goal.max_linear_vel,
                  w.goal.max_angular_vel, w.goal.lookahead_distance):
        assert isinstance(value, float)


@pytest.mark.parametrize('lat, lon, expected', [
    (0.0, 0.0, (WGS84_A, 0.0, 0.0)),
    (0.0, 90.0, (0.0, WGS84_A, 0.0)),  # catches swapped lat/lon axis order
    (90.0, 0.0, (0.0, 0.0, WGS84_B)),
])
def test_wgs_waypoint_is_converted_to_ecef(lat, lon, expected):
    w = Waypoint()
    success, _ = w.from_dict({'lat': lat, 'lon': lon}, 'earth')

    assert success
    assert w.goal.goal.header.frame_id == 'earth'
    position = w.goal.goal.pose.position
    assert (position.x, position.y, position.z) == pytest.approx(expected, abs=1e-3)


@pytest.mark.parametrize('waypoint_dict', [
    {},
    {'x': 1.0},
    {'lat': 50.0},
    {'max_linear_vel': 1.0},
])
def test_waypoint_without_coordinates_fails(waypoint_dict):
    success, msg = Waypoint().from_dict(waypoint_dict, 'map')

    assert not success
    assert msg


def test_route_keeps_waypoint_order():
    route = Route()
    success, _ = route.from_dict({'waypoints': {
        'start': {'x': 0.0, 'y': 0.0},
        'middle': {'x': 1.0, 'y': 1.0},
        'end': {'x': 2.0, 'y': 2.0},
    }}, 'map')

    assert success
    assert [w.name for w in route.waypoints] == ['start', 'middle', 'end']


def test_route_message():
    route = Route()
    route.from_dict({'waypoints': {
        'a': {'x': 0.0, 'y': 0.0, 'mandatory': True},
        'b': {'x': 5.0, 'y': 5.0, 'mandatory': False},
    }}, 'odom')

    msg = route.get_route()

    assert isinstance(msg, OperatorGoalArray)
    assert len(msg.goals) == 2
    assert msg.goals[1].goal.pose.position.x == pytest.approx(5.0)
    assert msg.goals[1].goal.header.frame_id == 'odom'
    assert [g.mandatory for g in msg.goals] == [True, False]


def test_route_reports_failed_waypoint_numbers():
    route = Route()
    success, msg = route.from_dict({'waypoints': {
        'ok': {'x': 0.0, 'y': 0.0},
        'bad_1': {'x': 1.0},
        'bad_2': {},
    }}, 'map')

    assert not success
    assert '[2, 3]' in msg
