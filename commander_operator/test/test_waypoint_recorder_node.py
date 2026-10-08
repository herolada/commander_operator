import math
import uuid

from commander_operator.waypoint_recorder_node import WaypointRecorder
from commander_operator.waypoints import load_routes, load_waypoints
from commander_operator_interfaces.srv import SaveWaypoints
from crl_commander_interfaces.msg import OperatorGoal
from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
import yaml

WGS84_A = 6378137.0  # equatorial radius [m]


@pytest.fixture
def recorder(tmp_path):
    namespace = '/test_waypoint_recorder_' + uuid.uuid4().hex[:8]
    rclpy.init(args=['--ros-args', '-r', f'__ns:={namespace}',
                     '-p', f'data_dir:={tmp_path}', '-p', 'tf_timeout:=0.1'])
    node = WaypointRecorder()

    # odom is shifted by (10, 0) in map, map sits on the equator at lon 0 with
    # x pointing east (= ECEF +y) and y pointing north (= ECEF +z).
    add_transform(node, 'map', 'odom', translation=(10.0, 0.0, 0.0))
    add_transform(node, 'earth', 'map', translation=(WGS84_A, 0.0, 0.0),
                  rotation=(0.5, 0.5, 0.5, 0.5))
    yield node

    node.destroy_node()
    rclpy.shutdown()


def add_transform(node, parent, child, translation, rotation=(0.0, 0.0, 0.0, 1.0)):
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
        translation
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = rotation
    node.tf_buffer.set_transform_static(t, 'test')


def goal(frame_id, x, y, **params):
    g = OperatorGoal(mandatory=params.pop('mandatory', True))
    for key, value in params.items():
        setattr(g, key, value)
        setattr(g, 'use_' + key, True)
    g.goal.header.frame_id = frame_id
    g.goal.pose.position.x = x
    g.goal.pose.position.y = y
    return g


def call(node, names, goals, wgs=False, route_name='', overwrite=False):
    request = SaveWaypoints.Request(
        wgs=wgs, route_name=route_name, overwrite=overwrite, names=names, goals=goals)
    return node.save_waypoints_cb(request, SaveWaypoints.Response())


def read_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def test_local_waypoint_is_transformed_to_map_frame(recorder, tmp_path):
    response = call(recorder, ['gate'], [goal('odom', 1.0, 2.0, max_linear_vel=1.2)])

    assert response.success, response.message
    assert response.file_path == str(tmp_path / 'waypoints.yaml')
    assert read_yaml(response.file_path) == {
        'frame_id': 'map',
        'waypoints': {'gate': {'x': 11.0, 'y': 2.0, 'max_linear_vel': 1.2}}}


def test_map_frame_parameter_is_used(recorder, tmp_path):
    recorder.map_frame = 'odom'

    response = call(recorder, ['gate'], [goal('map', 1.0, 2.0)])

    assert response.success, response.message
    assert read_yaml(response.file_path) == {
        'frame_id': 'odom', 'waypoints': {'gate': {'x': -9.0, 'y': 2.0}}}


def test_wgs_waypoint(recorder, tmp_path):
    # 1 km north of the map origin = 1 km along the meridian.
    response = call(recorder, ['north'], [goal('odom', -10.0, 1000.0)], wgs=True)

    assert response.success, response.message
    assert response.file_path == str(tmp_path / 'waypoints_wgs.yaml')
    entry = read_yaml(response.file_path)['waypoints']['north']
    assert set(entry) == {'lat', 'lon'}
    assert entry['lon'] == pytest.approx(0.0, abs=1e-8)
    assert entry['lat'] == pytest.approx(math.degrees(1000.0 / 6335439.0), rel=1e-3)


def test_route(recorder, tmp_path):
    goals = [goal('map', 0.0, 0.0), goal('map', 1.0, 1.0, mandatory=False, max_angular_vel=0.5)]

    response = call(recorder, ['start', 'waypoint_2'], goals, route_name='patrol')

    assert response.success, response.message
    assert response.file_path == str(tmp_path / 'routes' / 'patrol.yaml')
    data = read_yaml(response.file_path)
    assert data['frame_id'] == 'map'
    assert data['name'] == 'patrol'
    assert data['waypoints'] == {
        'start': {'x': 0.0, 'y': 0.0, 'mandatory': True},
        'waypoint_2': {'x': 1.0, 'y': 1.0, 'max_angular_vel': 0.5, 'mandatory': False}}


def test_saved_files_load_back(recorder, tmp_path):
    call(recorder, ['gate'], [goal('odom', 1.0, 2.0)])
    call(recorder, ['gate'], [goal('odom', 1.0, 2.0)], wgs=True)
    call(recorder, ['a', 'b'], [goal('map', 0.0, 0.0), goal('map', 5.0, 0.0)],
         route_name='r')
    call(recorder, ['a', 'b'], [goal('map', 0.0, 0.0), goal('map', 5.0, 0.0)],
         route_name='r', wgs=True)
    logger = recorder.get_logger()

    waypoints = load_waypoints(str(tmp_path / 'waypoints.yaml'), logger)
    waypoints_wgs = load_waypoints(str(tmp_path / 'waypoints_wgs.yaml'), logger, 'earth')
    routes = load_routes(str(tmp_path / 'routes'), logger)
    routes_wgs = load_routes(str(tmp_path / 'routes_wgs'), logger, 'earth')

    assert waypoints['gate'].goal.goal.pose.position.x == pytest.approx(11.0)
    # Back in ECEF: map origin is at (A, 0, 0), map x is ECEF y.
    p = waypoints_wgs['gate'].goal.goal.pose.position
    assert (p.x, p.y, p.z) == pytest.approx((WGS84_A, 11.0, 2.0), abs=0.01)
    assert [w.name for w in routes['r'].waypoints] == ['a', 'b']
    assert routes_wgs['r'].waypoints[1].goal.goal.pose.position.y == pytest.approx(5.0, abs=0.01)


@pytest.mark.parametrize('names, goals, route_name, match', [
    ([], [], '', 'No waypoints'),
    (['a'], [goal('map', 0, 0), goal('map', 1, 1)], 'r', 'names for'),
    (['bad name'], [goal('map', 0, 0)], '', 'Invalid waypoint name'),
    (['a', 'a'], [goal('map', 0, 0), goal('map', 1, 1)], 'r', 'unique'),
    (['a'], [goal('map', 0, 0)], '../r', 'Invalid route name'),
    (['a', 'b'], [goal('map', 0, 0), goal('map', 1, 1)], '', 'exactly one'),
    (['a'], [goal('', 0, 0)], '', 'no frame_id'),
    (['a'], [goal('unknown_frame', 0, 0)], '', 'unknown_frame'),
])
def test_invalid_requests(recorder, tmp_path, names, goals, route_name, match):
    response = call(recorder, names, goals, route_name=route_name)

    assert not response.success
    assert match in response.message
    assert list(tmp_path.iterdir()) == []


def test_duplicate_waypoint_needs_overwrite(recorder):
    call(recorder, ['gate'], [goal('map', 1.0, 1.0)])

    response = call(recorder, ['gate'], [goal('map', 2.0, 2.0)])
    assert not response.success
    assert 'overwrite' in response.message

    response = call(recorder, ['gate'], [goal('map', 2.0, 2.0)], overwrite=True)
    assert response.success
    assert read_yaml(response.file_path)['waypoints']['gate'] == {'x': 2.0, 'y': 2.0}


def test_reload_is_requested_when_available(recorder, monkeypatch):
    calls = []
    monkeypatch.setattr(recorder.reload_client, 'service_is_ready', lambda: True)
    monkeypatch.setattr(recorder.reload_client, 'call_async', calls.append)

    response = call(recorder, ['gate'], [goal('map', 1.0, 1.0)])

    assert response.success
    assert len(calls) == 1
    assert 'not reloaded' not in response.message


def test_missing_reload_service_is_reported(recorder):
    response = call(recorder, ['gate'], [goal('map', 1.0, 1.0)])

    assert response.success
    assert 'not reloaded' in response.message
