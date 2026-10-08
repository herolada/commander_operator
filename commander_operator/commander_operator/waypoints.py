"""Waypoint and route model plus the YAML reading/writing shared by the operator nodes."""

import glob
import os
import re

from crl_commander_interfaces.msg import OperatorGoal, OperatorGoalArray
import pyproj
import yaml

WGS_TO_ECEF = pyproj.Transformer.from_crs('EPSG:4326', 'EPSG:4978')
ECEF_TO_WGS = pyproj.Transformer.from_crs('EPSG:4978', 'EPSG:4326')

# Optional per-waypoint controller parameters, in the order they are written.
OPTIONAL_PARAMS = ('max_linear_vel', 'max_angular_vel', 'lookahead_distance')

# Decimal places used when writing coordinates (1 mm locally, ~1 mm in WGS).
LOCAL_DECIMALS = 3
WGS_DECIMALS = 8
PARAM_DECIMALS = 4

NAME_RE = re.compile(r'^[A-Za-z_][A-Za-z0-9_\-]*$')
DEFAULT_INDENT = '  '


class Waypoint:

    def __init__(self, name=None):
        # Only 'mandatory' has a reasonable default value, others need flags to indicate
        # whether they are being set or not.
        self.use_max_linear_vel = False
        self.use_max_angular_vel = False
        self.use_lookahead_distance = False
        self.max_linear_vel = -1.
        self.max_angular_vel = -1.
        self.lookahead_distance = -1.
        self.mandatory = True
        # self.x = 0.
        # self.y = 0.
        # self.frame_id = None
        self.goal = OperatorGoal()
        self.name = name

    def from_dict(self, waypoint_dict, frame_id):
        if 'max_linear_vel' in waypoint_dict:
            self.max_linear_vel = float(waypoint_dict['max_linear_vel'])
            self.use_max_linear_vel = True

        if 'max_angular_vel' in waypoint_dict:
            self.max_angular_vel = float(waypoint_dict['max_angular_vel'])
            self.use_max_angular_vel = True

        if 'lookahead_distance' in waypoint_dict:
            self.lookahead_distance = float(waypoint_dict['lookahead_distance'])
            self.use_lookahead_distance = True

        if 'mandatory' in waypoint_dict:
            self.mandatory = waypoint_dict['mandatory']

        if ('lat' in waypoint_dict) and ('lon' in waypoint_dict):
            # a wgs point
            lat = waypoint_dict['lat']
            lon = waypoint_dict['lon']
            ecef_x, ecef_y, ecef_z = WGS_TO_ECEF.transform(lat, lon, 0)
            self.x = ecef_x
            self.y = ecef_y
            self.z = ecef_z
        elif ('x' in waypoint_dict) and ('y' in waypoint_dict):
            self.x = float(waypoint_dict['x'])
            self.y = float(waypoint_dict['y'])
            self.z = 0.0
        else:
            return False, 'Did not find neither lat,lon nor x,y.'

        self.frame_id = frame_id

        self.goal = OperatorGoal()

        self.goal.max_linear_vel = self.max_linear_vel
        self.goal.use_max_linear_vel = self.use_max_linear_vel

        self.goal.max_angular_vel = self.max_angular_vel
        self.goal.use_max_angular_vel = self.use_max_angular_vel

        self.goal.lookahead_distance = self.lookahead_distance
        self.goal.use_lookahead_distance = self.use_lookahead_distance

        self.goal.mandatory = self.mandatory

        self.goal.goal.header.frame_id = self.frame_id
        self.goal.goal.pose.position.x = self.x
        self.goal.goal.pose.position.y = self.y
        self.goal.goal.pose.position.z = self.z

        return True, 'Success.'


class Route:

    def __init__(self):
        self.waypoints = []

    def get_route(self, selected_waypoints=None):
        route = OperatorGoalArray()
        if selected_waypoints is not None:
            raise NotImplementedError('to be done')
        else:
            route.goals = [w.goal for w in self.waypoints]

        return route

    def from_dict(self, waypoint_dict, frame_id):
        failed_waypoints = []
        failed_waypoints_msgs = []

        for i, name in enumerate(waypoint_dict['waypoints'].keys()):
            waypoint = waypoint_dict['waypoints'][name]
            w = Waypoint(name)
            success, msg = w.from_dict(waypoint, frame_id)
            if not success:
                failed_waypoints.append(i + 1)
                failed_waypoints_msgs.append(msg)
                continue

            self.waypoints.append(w)

        if len(failed_waypoints):
            return False, (f'Failed to load waypoints number: {failed_waypoints}. '
                           f'For following reasons: {failed_waypoints_msgs}.')

        return True, 'Success.'


# ── Loading ──────────────────────────────────────────────────────────────────

def load_waypoints(file_path, logger, default_frame_id=None):
    waypoints = {}

    if not os.path.exists(file_path):
        logger.warning(
            f'File {file_path} does not exist. Cannot load waypoints from it.')
        return waypoints

    with open(file_path, 'r') as f:
        waypoints_yaml = yaml.safe_load(f)
        for waypoint_name in waypoints_yaml['waypoints'].keys():
            waypoint_dict = waypoints_yaml['waypoints'][waypoint_name]

            frame_id = None
            if 'frame_id' in waypoints_yaml:
                frame_id = waypoints_yaml['frame_id']
            elif default_frame_id is not None:
                frame_id = default_frame_id
            else:
                logger.error(
                    'Frame_id not provided for non-wgs waypoints! Cannot proceed!')
                continue

            w = Waypoint(waypoint_name)
            success, msg = w.from_dict(waypoint_dict, frame_id)

            if success:
                if waypoint_name in waypoints.keys():
                    logger.warning(
                        f"Detected duplicate waypoint '{waypoint_name}' in '{file_path}', "
                        'overwriting the previous one!')
                waypoints[waypoint_name] = w
            else:
                logger.error("Failed to load waypoint: '" + msg + "'")

    return waypoints


def load_routes(folder_path, logger, default_frame_id=None):
    routes = {}
    route_files = glob.glob(os.path.join(folder_path, '*.yaml'))

    if not os.path.exists(folder_path):
        logger.warning(
            f'Folder {folder_path} does not exist. Cannot load routes from it.')
        return routes

    for route_file in route_files:
        with open(route_file, 'r') as f:
            waypoints_yaml = yaml.safe_load(f)

            frame_id = None
            if 'frame_id' in waypoints_yaml:
                frame_id = waypoints_yaml['frame_id']
            elif default_frame_id is not None:
                frame_id = default_frame_id
            else:
                logger.error(
                    'Frame_id not provided for non-wgs routes! Cannot proceed!')
                continue

            route = Route()
            success, msg = route.from_dict(waypoints_yaml, frame_id)

            if success:
                name = waypoints_yaml['name']
                if name in routes.keys():
                    logger.warning(
                        f"Detected duplicate route '{name}' in '{folder_path}', "
                        'overwriting the previous one!')
                routes[name] = route
            else:
                logger.error("Failed to load route: '" + msg + "'")

    return routes


# ── Writing ──────────────────────────────────────────────────────────────────
#
# Files are written in the hand-written style of the shipped examples: one flow
# mapping per waypoint, with the mappings aligned in a column. New waypoints are
# appended as text so that comments and formatting of the existing file survive;
# only when that is not possible (overwriting an entry, unusual file layout) is
# the whole file regenerated, keeping just its leading comment block.

class WaypointFileError(ValueError):
    """Raised when a waypoint or route cannot be written."""


def is_valid_name(name):
    """Return True if the name is usable as a waypoint/route name (and YAML key)."""
    if not NAME_RE.match(name):
        return False
    # Reject names YAML would not read back as the same string (true, null, on, ...).
    return yaml.safe_load(name) == name


def make_entry(goal, coordinates, include_mandatory):
    """
    Build the dict written for one waypoint.

    `coordinates` is an ordered dict of the position fields ({'x', 'y'} or
    {'lat', 'lon'}), the optional parameters are taken from the OperatorGoal.
    """
    entry = dict(coordinates)
    for param in OPTIONAL_PARAMS:
        if getattr(goal, 'use_' + param):
            entry[param] = round(float(getattr(goal, param)), PARAM_DECIMALS)
    if include_mandatory:
        entry['mandatory'] = bool(goal.mandatory)
    return entry


def format_value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, float):
        return repr(value)
    return str(value)


def format_entry(name, entry, brace_column=0, indent=DEFAULT_INDENT):
    """Format one waypoint as `  name:   {key: value, ...}`, aligning '{' to brace_column."""
    prefix = f'{indent}{name}:'
    padding = max(1, brace_column - len(prefix))
    body = ', '.join(f'{key}: {format_value(value)}' for key, value in entry.items())
    return prefix + ' ' * padding + '{' + body + '}'


def _brace_column(names, indent=DEFAULT_INDENT):
    """Column where '{' starts when all the names are aligned with a single space."""
    return max((len(indent) + len(name) + 2 for name in names), default=0)


def format_document(top, entries, header=''):
    """
    Format a whole waypoints/route file.

    `top` holds the top level keys written before `waypoints:` (frame_id, name),
    `entries` is a list of (name, entry dict) pairs.
    """
    lines = [header.rstrip('\n')] if header.strip() else []
    for key, value in top.items():
        lines.append(yaml.safe_dump({key: value}, default_flow_style=False).rstrip('\n'))
    if entries:
        lines.append('waypoints:')
        column = _brace_column(name for name, _ in entries)
        lines += [format_entry(name, entry, column) for name, entry in entries]
    else:
        lines.append('waypoints: {}')
    return '\n'.join(lines) + '\n'


def leading_comments(text):
    """Return the comment block at the top of a file (kept when it is regenerated)."""
    header = []
    for line in text.splitlines():
        if line.startswith('#') or (header and not line.strip()):
            header.append(line)
        else:
            break
    return '\n'.join(header).rstrip('\n')


_ENTRY_LINE_RE = re.compile(r'^(\s+)[^\s#][^:]*:\s*\{')


def _existing_layout(text):
    """Return (indent, brace column) used by the flow-style entries of a file."""
    indent, column = DEFAULT_INDENT, 0
    for line in text.splitlines():
        match = _ENTRY_LINE_RE.match(line)
        if match:
            indent = match.group(1)
            column = max(column, line.index('{'))
    return indent, column


def append_entries(text, data, entries):
    """
    Return `text` with the entries appended to its `waypoints` mapping.

    Returns None when plain appending would not produce the expected content
    (e.g. `waypoints` is not the last key or is written in flow style).
    """
    indent, column = _existing_layout(text)
    lines = [format_entry(name, entry, column, indent) for name, entry in entries]
    new_text = text.rstrip() + '\n' + '\n'.join(lines) + '\n'

    expected = dict(data)
    expected['waypoints'] = dict(data.get('waypoints') or {})
    expected['waypoints'].update(entries)
    try:
        if yaml.safe_load(new_text) != expected:
            return None
    except yaml.YAMLError:
        return None
    # Order matters for routes and dict equality ignores it.
    if list(yaml.safe_load(new_text)['waypoints']) != list(expected['waypoints']):
        return None
    return new_text


def _read(path):
    with open(path, 'r') as f:
        text = f.read()
    try:
        data = yaml.safe_load(text) or {}
    except yaml.YAMLError as e:
        raise WaypointFileError(f"Cannot parse '{path}': {e}")
    if not isinstance(data, dict) or not isinstance(data.get('waypoints') or {}, dict):
        raise WaypointFileError(f"'{path}' does not contain a 'waypoints' mapping.")
    return text, data


def _write(path, text):
    # Write through a temporary file so a failure never leaves a half written file.
    # Writing into the real path keeps symlinks (colcon --symlink-install) intact.
    path = os.path.realpath(path)
    tmp_path = path + '.tmp'
    with open(tmp_path, 'w') as f:
        f.write(text)
    os.replace(tmp_path, path)


def save_waypoint(path, name, entry, frame_id=None, overwrite=False):
    """
    Add a waypoint to a waypoints file (waypoints.yaml / waypoints_wgs.yaml).

    `frame_id` is required for local files and must match the one in the file,
    it is None for WGS files. An existing waypoint of the same name is replaced
    only with `overwrite`.
    """
    top = {} if frame_id is None else {'frame_id': frame_id}

    if not os.path.exists(path):
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        _write(path, format_document(top, [(name, entry)]))
        return

    text, data = _read(path)

    if frame_id is not None and data.get('frame_id') != frame_id:
        raise WaypointFileError(
            f"'{path}' has frame_id '{data.get('frame_id')}', "
            f"but the waypoint is in '{frame_id}'.")

    waypoints = dict(data.get('waypoints') or {})
    if name in waypoints:
        if not overwrite:
            raise WaypointFileError(
                f"Waypoint '{name}' already exists in '{path}'. Enable overwrite to replace it.")
        waypoints[name] = entry
        file_top = {key: value for key, value in data.items() if key != 'waypoints'}
        _write(path, format_document(
            file_top, list(waypoints.items()), leading_comments(text)))
        return

    new_text = append_entries(text, data, [(name, entry)])
    if new_text is None:
        waypoints[name] = entry
        file_top = {key: value for key, value in data.items() if key != 'waypoints'}
        new_text = format_document(file_top, list(waypoints.items()), leading_comments(text))
    _write(path, new_text)


def save_route(folder_path, route_name, entries, frame_id=None, overwrite=False):
    """
    Write a route into `folder_path/<route_name>.yaml` and return the file path.

    `frame_id` is required for local routes, None for WGS routes. An existing route
    file is replaced only with `overwrite`, a route of the same name stored in a
    different file is always refused (the loader would silently keep just one).
    """
    path = os.path.join(folder_path, route_name + '.yaml')

    for other in glob.glob(os.path.join(folder_path, '*.yaml')):
        if os.path.abspath(other) == os.path.abspath(path):
            continue
        try:
            with open(other, 'r') as f:
                other_name = (yaml.safe_load(f) or {}).get('name')
        except (yaml.YAMLError, AttributeError):
            continue
        if other_name == route_name:
            raise WaypointFileError(
                f"Route '{route_name}' is already defined in '{other}'.")

    header = ''
    if os.path.exists(path):
        if not overwrite:
            raise WaypointFileError(
                f"Route file '{path}' already exists. Enable overwrite to replace it.")
        with open(path, 'r') as f:
            header = leading_comments(f.read())

    top = {} if frame_id is None else {'frame_id': frame_id}
    top['name'] = route_name

    os.makedirs(folder_path, exist_ok=True)
    _write(path, format_document(top, entries, header))
    return path
