import glob
import os
import time
from unittest import mock
import uuid

from commander_operator.commander_operator_node import CommanderOperator
from commander_operator_interfaces.msg import OperatorRequest
from crl_commander_interfaces.msg import OperatorGoal, OperatorGoalArray
from crl_commander_interfaces.srv import SwitchMode
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.task import Future

SHIPPED_DATA_DIR = os.path.join(os.path.dirname(__file__), '..', 'data')

DEFAULT_FILES = {
    'waypoints.yaml': """
frame_id: map
waypoints:
  home: {x: 0.0, y: 0.0}
  dock: {x: 3.2, y: -1.0, max_linear_vel: 1.2}
""",
    'waypoints_wgs.yaml': """
waypoints:
  null_island: {lat: 0.0, lon: 0.0}
""",
    'routes/patrol.yaml': """
frame_id: odom
name: patrol
waypoints:
  a: {x: 0.0, y: 0.0}
  b: {x: 1.0, y: 1.0, mandatory: false}
  c: {x: 2.0, y: 2.0}
""",
    'routes_wgs/wgs_patrol.yaml': """
name: wgs_patrol
waypoints:
  a: {lat: 50.00, lon: 14.00}
  b: {lat: 50.01, lon: 14.01}
""",
}


@pytest.fixture
def start_node(tmp_path):
    """
    Return a factory that writes data files and starts a CommanderOperator on them.

    Every test runs in its own random namespace, so it can never talk to a
    real commander that happens to run on the same ROS domain.
    """
    nodes = []

    def _start(files=None, data_dir=None, params=()):
        if data_dir is None:
            data_dir = str(tmp_path)
            for rel_path, content in (DEFAULT_FILES if files is None else files).items():
                path = tmp_path / rel_path
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)

        namespace = '/test_commander_operator_' + uuid.uuid4().hex[:8]
        args = ['--ros-args', '-r', f'__ns:={namespace}', '-p', f'data_dir:={data_dir}']
        for param in params:
            args += ['-p', param]
        rclpy.init(args=args)

        node = CommanderOperator()
        nodes.append(node)
        return node

    yield _start

    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


# ── Loading ──────────────────────────────────────────────────────────────────

def test_loads_all_sources(start_node):
    node = start_node()

    assert set(node.waypoints) == {'home', 'dock'}
    assert set(node.waypoints_wgs) == {'null_island'}
    assert set(node.routes) == {'patrol'}
    assert set(node.routes_wgs) == {'wgs_patrol'}

    assert node.waypoints['dock'].goal.goal.header.frame_id == 'map'
    assert node.waypoints_wgs['null_island'].goal.goal.header.frame_id == 'earth'
    assert len(node.routes['patrol'].waypoints) == 3
    assert node.routes['patrol'].waypoints[0].goal.goal.header.frame_id == 'odom'
    assert node.routes_wgs['wgs_patrol'].waypoints[0].goal.goal.header.frame_id == 'earth'


def test_ecef_frame_parameter_is_used_for_wgs(start_node):
    node = start_node(params=['ecef_frame:=FP_ECEF'])

    assert node.waypoints_wgs['null_island'].goal.goal.header.frame_id == 'FP_ECEF'
    for w in node.routes_wgs['wgs_patrol'].waypoints:
        assert w.goal.goal.header.frame_id == 'FP_ECEF'


def test_shipped_example_data_loads(start_node):
    node = start_node(data_dir=SHIPPED_DATA_DIR)

    assert set(node.waypoints) == {'home', 'dock', 'station_a'}
    assert set(node.waypoints_wgs) == {'home', 'checkpoint_1', 'checkpoint_2'}
    assert len(node.routes['example_route'].waypoints) == 6
    assert len(node.routes_wgs['example_route'].waypoints) == 6


def test_missing_files_give_empty_collections(start_node):
    node = start_node(files={})

    assert node.waypoints == {}
    assert node.waypoints_wgs == {}
    assert node.routes == {}
    assert node.routes_wgs == {}


def test_local_waypoints_without_frame_id_are_skipped(start_node):
    files = dict(DEFAULT_FILES)
    files['waypoints.yaml'] = """
waypoints:
  home: {x: 0.0, y: 0.0}
"""
    node = start_node(files=files)

    assert node.waypoints == {}


def test_invalid_waypoint_is_skipped(start_node):
    files = dict(DEFAULT_FILES)
    files['waypoints.yaml'] = """
frame_id: map
waypoints:
  good: {x: 1.0, y: 1.0}
  bad: {x: 1.0}
"""
    node = start_node(files=files)

    assert set(node.waypoints) == {'good'}


@pytest.mark.parametrize('reverse', [False, True])
def test_route_frame_id_is_per_file(start_node, monkeypatch, reverse):
    # A file without frame_id must not inherit the frame of another file. The
    # bug only shows in some file orders, so both orders are forced.
    real_glob = glob.glob
    monkeypatch.setattr(
        glob, 'glob', lambda pattern: sorted(real_glob(pattern), reverse=reverse))
    files = {
        'routes/a.yaml': 'frame_id: odom\nname: a\nwaypoints:\n  p: {x: 0.0, y: 0.0}\n',
        'routes/b.yaml': 'name: b\nwaypoints:\n  p: {x: 0.0, y: 0.0}\n',
        'routes/c.yaml': 'frame_id: map\nname: c\nwaypoints:\n  p: {x: 0.0, y: 0.0}\n',
    }
    node = start_node(files=files)

    assert set(node.routes) == {'a', 'c'}
    assert node.routes['a'].waypoints[0].goal.goal.header.frame_id == 'odom'
    assert node.routes['c'].waypoints[0].goal.goal.header.frame_id == 'map'


def test_route_with_invalid_waypoint_is_rejected(start_node):
    files = {
        'routes/broken.yaml': """
frame_id: map
name: broken
waypoints:
  ok: {x: 0.0, y: 0.0}
  bad: {y: 1.0}
""",
    }
    node = start_node(files=files)

    assert node.routes == {}


def test_non_yaml_files_in_route_folder_are_ignored(start_node):
    files = dict(DEFAULT_FILES)
    files['routes/notes.txt'] = 'not: [valid yaml'
    node = start_node(files=files)

    assert set(node.routes) == {'patrol'}


# ── Goal lookup ──────────────────────────────────────────────────────────────

@pytest.mark.parametrize('goal, route, wgs, collection, mode', [
    ('dock', False, False, 'waypoints', SwitchMode.Request.MODE_GOTO),
    ('null_island', False, True, 'waypoints_wgs', SwitchMode.Request.MODE_GOTO),
    ('patrol', True, False, 'routes', SwitchMode.Request.MODE_SEQUENCE),
    ('wgs_patrol', True, True, 'routes_wgs', SwitchMode.Request.MODE_SEQUENCE),
])
def test_goal_lookup(start_node, goal, route, wgs, collection, mode):
    node = start_node()

    goal_object, request = node.get_goal_object_and_request(goal, route, wgs)

    assert goal_object is getattr(node, collection)[goal]
    assert request.mode == mode


@pytest.mark.parametrize('goal, route, wgs', [
    ('nonexistent', False, False),
    ('dock', False, True),  # local waypoint asked for as wgs
    ('dock', True, False),  # waypoint asked for as route
    ('patrol', False, False),  # route asked for as waypoint
])
def test_goal_lookup_unknown(start_node, goal, route, wgs):
    node = start_node()

    goal_object, _ = node.get_goal_object_and_request(goal, route, wgs)

    assert goal_object is None


# ── Request callback (publishers and client mocked) ──────────────────────────

def mock_outputs(node, service_ready=True):
    manager = mock.Mock()
    node.operator_goal_pub = manager.goal_pub
    node.operator_sequence_pub = manager.sequence_pub
    node.commander_switch_mode_client = manager.client
    manager.client.service_is_ready.return_value = service_ready
    return manager


def call_names(manager):
    return [c[0] for c in manager.mock_calls]


def test_waypoint_request_publishes_goal_before_switching_mode(start_node):
    node = start_node()
    manager = mock_outputs(node)

    node.operator_request_cb(OperatorRequest(goal='dock'))

    manager.goal_pub.publish.assert_called_once_with(node.waypoints['dock'].goal)
    manager.sequence_pub.publish.assert_not_called()
    request = manager.client.call_async.call_args.args[0]
    assert request.mode == SwitchMode.Request.MODE_GOTO
    names = call_names(manager)
    assert names.index('goal_pub.publish') < names.index('client.call_async')


def test_route_request_publishes_sequence_before_switching_mode(start_node):
    node = start_node()
    manager = mock_outputs(node)

    node.operator_request_cb(OperatorRequest(goal='patrol', route=True))

    manager.goal_pub.publish.assert_not_called()
    sequence = manager.sequence_pub.publish.call_args.args[0]
    assert isinstance(sequence, OperatorGoalArray)
    assert len(sequence.goals) == 3
    request = manager.client.call_async.call_args.args[0]
    assert request.mode == SwitchMode.Request.MODE_SEQUENCE
    names = call_names(manager)
    assert names.index('sequence_pub.publish') < names.index('client.call_async')


def test_unknown_goal_does_nothing(start_node):
    node = start_node()
    manager = mock_outputs(node)

    node.operator_request_cb(OperatorRequest(goal='nonexistent'))

    manager.goal_pub.publish.assert_not_called()
    manager.sequence_pub.publish.assert_not_called()
    manager.client.call_async.assert_not_called()


def test_request_dropped_when_commander_unavailable(start_node):
    node = start_node()
    manager = mock_outputs(node, service_ready=False)

    node.operator_request_cb(OperatorRequest(goal='dock'))

    manager.goal_pub.publish.assert_not_called()
    manager.client.call_async.assert_not_called()


def test_each_request_gets_a_new_id(start_node):
    node = start_node()
    mock_outputs(node)

    node.operator_request_cb(OperatorRequest(goal='dock'))
    node.operator_request_cb(OperatorRequest(goal='home'))

    assert node.request_id == 2


# ── switch_mode response handling ────────────────────────────────────────────

def finished_future(response=None, exception=None):
    future = Future()
    if exception is not None:
        future.set_exception(exception)
    else:
        future.set_result(response)
    return future


@pytest.fixture
def logger(start_node, monkeypatch):
    node = start_node()
    node.request_id = 1
    log = mock.Mock()
    monkeypatch.setattr(node, 'get_logger', lambda: log)
    return node, log


def test_switch_mode_success_is_logged(logger):
    node, log = logger
    response = SwitchMode.Response(success=True, message='ok')

    node.switch_mode_done(finished_future(response), 1, 'goto', 'dock')

    log.info.assert_called_once()
    log.error.assert_not_called()


def test_switch_mode_rejection_is_logged(logger):
    node, log = logger
    response = SwitchMode.Response(success=False, message='Unknown mode')

    node.switch_mode_done(finished_future(response), 1, 'goto', 'dock')

    log.error.assert_called_once()
    assert 'Unknown mode' in log.error.call_args.args[0]


def test_switch_mode_exception_is_logged(logger):
    node, log = logger

    node.switch_mode_done(finished_future(exception=RuntimeError('boom')), 1, 'goto', 'dock')

    log.error.assert_called_once()
    assert 'boom' in log.error.call_args.args[0]


def test_superseded_switch_mode_response_is_ignored(logger):
    node, log = logger
    node.request_id = 2  # a newer request was sent meanwhile
    response = SwitchMode.Response(success=False, message='stale')

    node.switch_mode_done(finished_future(response), 1, 'goto', 'dock')

    log.info.assert_not_called()
    log.error.assert_not_called()


# ── End to end over ROS, against a fake commander ────────────────────────────

class FakeCommander(Node):

    def __init__(self):
        super().__init__('fake_commander')
        self.mode_requests = []
        self.goals = []
        self.sequences = []
        reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_service(SwitchMode, 'switch_mode', self.switch_mode_cb)
        # Volatile subscriptions, like crl_commander.
        self.create_subscription(OperatorGoal, 'operator_goal', self.goals.append, reliable)
        self.create_subscription(
            OperatorGoalArray, 'operator_sequence', self.sequences.append, reliable)
        self.request_pub = self.create_publisher(OperatorRequest, 'goal', reliable)

    def switch_mode_cb(self, request, response):
        self.mode_requests.append(request.mode)
        response.success = True
        response.message = 'ok'
        return response


def spin_until(executor, condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    return condition()


@pytest.fixture
def ros_system(start_node):
    node = start_node()
    commander = FakeCommander()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(commander)

    connected = spin_until(executor, lambda: (
        node.commander_switch_mode_client.service_is_ready()
        and commander.request_pub.get_subscription_count() > 0
        and node.operator_goal_pub.get_subscription_count() > 0
        and node.operator_sequence_pub.get_subscription_count() > 0))
    assert connected, 'nodes did not discover each other'

    yield node, commander, executor

    executor.shutdown()
    commander.destroy_node()


def test_end_to_end_waypoint(ros_system):
    node, commander, executor = ros_system

    commander.request_pub.publish(OperatorRequest(goal='dock'))

    assert spin_until(executor, lambda: commander.goals and commander.mode_requests)
    assert commander.mode_requests == [SwitchMode.Request.MODE_GOTO]
    goal = commander.goals[0]
    assert goal.goal.header.frame_id == 'map'
    assert goal.goal.pose.position.x == pytest.approx(3.2)
    assert goal.use_max_linear_vel
    assert goal.max_linear_vel == pytest.approx(1.2)
    assert commander.sequences == []


def test_end_to_end_route(ros_system):
    node, commander, executor = ros_system

    commander.request_pub.publish(OperatorRequest(goal='patrol', route=True))

    assert spin_until(executor, lambda: commander.sequences and commander.mode_requests)
    assert commander.mode_requests == [SwitchMode.Request.MODE_SEQUENCE]
    sequence = commander.sequences[0]
    assert [g.goal.pose.position.x for g in sequence.goals] == pytest.approx([0.0, 1.0, 2.0])
    assert [g.mandatory for g in sequence.goals] == [True, False, True]
    assert commander.goals == []


def test_end_to_end_unknown_goal_sends_nothing(ros_system):
    node, commander, executor = ros_system

    commander.request_pub.publish(OperatorRequest(goal='nonexistent'))

    # Give the request time to arrive and be handled, then check nothing came back.
    spin_until(executor, lambda: False, timeout=1.0)
    assert commander.goals == []
    assert commander.sequences == []
    assert commander.mode_requests == []
