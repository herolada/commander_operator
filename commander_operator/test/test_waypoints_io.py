from commander_operator.waypoints import (
    format_entry, is_valid_name, make_entry, save_route, save_waypoint, WaypointFileError)
from crl_commander_interfaces.msg import OperatorGoal
import pytest
import yaml


def read_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


# ── Names ────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('name', ['home', 'waypoint_1', 'station-A', '_x'])
def test_valid_names(name):
    assert is_valid_name(name)


@pytest.mark.parametrize('name', [
    '', '1st', 'with space', 'a:b', '../escape', 'true', 'null', 'on', 'No', '-x'])
def test_invalid_names(name):
    assert not is_valid_name(name)


# ── Entries ──────────────────────────────────────────────────────────────────

def test_entry_contains_only_set_params():
    goal = OperatorGoal(max_linear_vel=1.2, use_max_linear_vel=True,
                        max_angular_vel=5.0, use_max_angular_vel=False,
                        mandatory=False)

    entry = make_entry(goal, {'x': 1.0, 'y': 2.0}, include_mandatory=False)

    assert entry == {'x': 1.0, 'y': 2.0, 'max_linear_vel': 1.2}
    assert list(entry) == ['x', 'y', 'max_linear_vel']


def test_entry_mandatory_for_routes():
    entry = make_entry(OperatorGoal(mandatory=False), {'x': 1.0, 'y': 2.0}, True)

    assert entry['mandatory'] is False


def test_format_entry_aligns_and_round_trips():
    line = format_entry('dock', {'x': 3.2, 'y': -1.0, 'mandatory': True}, brace_column=12)

    assert line == '  dock:     {x: 3.2, y: -1.0, mandatory: true}'
    assert yaml.safe_load(line.strip()) == {'dock': {'x': 3.2, 'y': -1.0, 'mandatory': True}}


# ── Waypoints file ───────────────────────────────────────────────────────────

EXISTING = """\
# some comment that must survive
frame_id: map
waypoints:
  home:     {x: 0.0, y: 0.0, max_linear_vel: 1.2}
  dock:     {x: 3.2, y: -1.0}"""


def test_append_keeps_existing_text(tmp_path):
    path = tmp_path / 'waypoints.yaml'
    path.write_text(EXISTING)

    save_waypoint(str(path), 'gate', {'x': 1.5, 'y': 2.5}, 'map')

    assert path.read_text() == EXISTING + '\n  gate:     {x: 1.5, y: 2.5}\n'
    assert list(read_yaml(path)['waypoints']) == ['home', 'dock', 'gate']


def test_long_name_is_appended_with_single_space(tmp_path):
    path = tmp_path / 'waypoints.yaml'
    path.write_text(EXISTING)

    save_waypoint(str(path), 'a_very_long_name', {'x': 1.0, 'y': 2.0}, 'map')

    assert path.read_text().endswith('\n  a_very_long_name: {x: 1.0, y: 2.0}\n')


def test_duplicate_name_is_refused(tmp_path):
    path = tmp_path / 'waypoints.yaml'
    path.write_text(EXISTING)

    with pytest.raises(WaypointFileError, match='already exists'):
        save_waypoint(str(path), 'dock', {'x': 9.0, 'y': 9.0}, 'map')
    assert path.read_text() == EXISTING


def test_duplicate_name_is_overwritten_in_place(tmp_path):
    path = tmp_path / 'waypoints.yaml'
    path.write_text(EXISTING)

    save_waypoint(str(path), 'home', {'x': 9.0, 'y': 9.0}, 'map', overwrite=True)

    text = path.read_text()
    data = read_yaml(path)
    assert text.startswith('# some comment that must survive\n')
    assert data['frame_id'] == 'map'
    assert list(data['waypoints']) == ['home', 'dock']
    assert data['waypoints']['home'] == {'x': 9.0, 'y': 9.0}


def test_frame_mismatch_is_refused(tmp_path):
    path = tmp_path / 'waypoints.yaml'
    path.write_text(EXISTING)

    with pytest.raises(WaypointFileError, match='frame_id'):
        save_waypoint(str(path), 'gate', {'x': 1.0, 'y': 1.0}, 'odom')


def test_missing_file_is_created(tmp_path):
    path = tmp_path / 'sub' / 'waypoints_wgs.yaml'

    save_waypoint(str(path), 'home', {'lat': 50.1, 'lon': 14.2}, None)

    assert read_yaml(path) == {'waypoints': {'home': {'lat': 50.1, 'lon': 14.2}}}


@pytest.mark.parametrize('content', [
    'frame_id: map\nwaypoints: {}\n',
    'frame_id: map\nwaypoints:\n',
    'waypoints:\n  home: {x: 0.0, y: 0.0}\nframe_id: map\n',  # waypoints not last
    'frame_id: map\nwaypoints: {home: {x: 0.0, y: 0.0}}\n',  # flow style
])
def test_unusual_layouts_are_regenerated(tmp_path, content):
    path = tmp_path / 'waypoints.yaml'
    path.write_text(content)
    before = read_yaml(path)['waypoints'] or {}

    save_waypoint(str(path), 'gate', {'x': 1.0, 'y': 2.0}, 'map')

    data = read_yaml(path)
    assert data['frame_id'] == 'map'
    assert list(data['waypoints']) == list(before) + ['gate']
    assert data['waypoints']['gate'] == {'x': 1.0, 'y': 2.0}


def test_symlinked_file_is_written_through(tmp_path):
    real = tmp_path / 'src_waypoints.yaml'
    real.write_text(EXISTING)
    link = tmp_path / 'waypoints.yaml'
    link.symlink_to(real)

    save_waypoint(str(link), 'gate', {'x': 1.0, 'y': 2.0}, 'map')

    assert link.is_symlink()
    assert 'gate' in read_yaml(real)['waypoints']


# ── Routes ───────────────────────────────────────────────────────────────────

ENTRIES = [
    ('start', {'x': 0.0, 'y': 0.0, 'mandatory': True}),
    ('waypoint_2', {'x': 1.0, 'y': 1.0, 'max_linear_vel': 1.5, 'mandatory': False}),
]


def test_route_is_written(tmp_path):
    path = save_route(str(tmp_path / 'routes'), 'patrol', ENTRIES, 'map')

    assert path == str(tmp_path / 'routes' / 'patrol.yaml')
    assert (tmp_path / 'routes' / 'patrol.yaml').read_text() == (
        'frame_id: map\n'
        'name: patrol\n'
        'waypoints:\n'
        '  start:      {x: 0.0, y: 0.0, mandatory: true}\n'
        '  waypoint_2: {x: 1.0, y: 1.0, max_linear_vel: 1.5, mandatory: false}\n')


def test_wgs_route_has_no_frame_id(tmp_path):
    path = save_route(str(tmp_path), 'patrol', [('a', {'lat': 50.0, 'lon': 14.0})], None)

    assert read_yaml(path) == {'name': 'patrol', 'waypoints': {'a': {'lat': 50.0, 'lon': 14.0}}}


def test_existing_route_needs_overwrite(tmp_path):
    save_route(str(tmp_path), 'patrol', ENTRIES, 'map')

    with pytest.raises(WaypointFileError, match='already exists'):
        save_route(str(tmp_path), 'patrol', ENTRIES[:1], 'map')
    assert len(read_yaml(tmp_path / 'patrol.yaml')['waypoints']) == 2

    save_route(str(tmp_path), 'patrol', ENTRIES[:1], 'map', overwrite=True)
    assert len(read_yaml(tmp_path / 'patrol.yaml')['waypoints']) == 1


def test_route_name_used_by_other_file_is_refused(tmp_path):
    (tmp_path / 'other_file.yaml').write_text(
        'frame_id: map\nname: patrol\nwaypoints:\n  a: {x: 0.0, y: 0.0}\n')

    with pytest.raises(WaypointFileError, match='other_file.yaml'):
        save_route(str(tmp_path), 'patrol', ENTRIES, 'map', overwrite=True)
