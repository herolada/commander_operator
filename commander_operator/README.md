## Commander operator

This package/node functions as an interface between the human operator and the commander node. The commander node fulfils requests for the robot to go to a point or to go through a sequence of points. The human operator then may want to go to a specific semantically significant point (e.g. home, charging station, command center...) or a predefined route (e.g. one of multiple security routes). This package/node shall bridge this gap between semantical point and actual coordinates in the robot's internal frame.

### Defining points

In general we use `yaml` and for local points, besides the name, we define the `frame_id, x, y`. For global points we use WGS and define `lat` and `lon`. Furthermore, we *can* also define the maximum lin/ang velocities and lookahead distance for the controller to get there (if not defined, default values of the controller are used).

#### Local frame points

Store each as a separate *entry* in a `data/waypoints.yaml` file.

```
frame_id: map
waypoints:
  home:     {x: 0.0, y: 0.0, max_linear_vel: 1.2, max_angular_vel: 1.2, lookahead_distance: 1.5}
  dock:     {x: 3.2, y: -1.0, max_linear_vel: 1.2}
  station_a: {x: 5.0, y: 2.5}
```

#### Global frame points (WGS coordinates)

Store each as a separate *entry* in a `data/waypoints_wgs.yaml` file.

```
# frame_id is not defined, as the robot does not require specifically a wgs frame, but any global frame will do
waypoints:
  home:         {lat: 50.00, lon: 14.00, max_linear_vel: 1.2, max_angular_vel: 1.2, lookahead_distance: 1.5}
  checkpoint_1: {lat: 50.01, lon: 14.01}
  checkpoint_2: {lat: 50.02, lon: 14.02, lookahead_distance: 1.5}
```

### Defining routes (point sequences)

In general we use `yaml` and for sequences of local points, we define the `frame_id, waypoint_name, x, y`. For global points we use WGS and define `lat` and `lon`. Furthermore, we *can* also define the maximum lin/ang velocities and lookahead distance for the controller to get there (if not defined, default values of the controller are used).

#### Local frame

Store each as a separate *file* in a `data/routes/{route_name}.yaml` file.

```
frame_id: map
name: example_route
waypoints:
  start:        {x: 0.0, y: 0.0, max_linear_vel: 1.2, mandatory: true}
  waypoint_1:   {x: 1.0, y: 1.0, max_linear_vel: 1.5, mandatory: false}
  waypoint_2:   {x: 2.0, y: 2.0, max_linear_vel: 1.5, mandatory: false}
  checkpoint_a: {x: 3.0, y: 3.0, max_linear_vel: 1.0, mandatory: true}
  waypoint_3:   {x: 4.0, y: 4.0, max_linear_vel: 1.0, mandatory: false}
  waypoint_4:   {x: 5.0, y: 5.0, max_linear_vel: 1.2, max_angular_vel: 1.2, lookahead_distance: 1.5, mandatory: false}
  ...
```

#### Global frame (WGS coordinates)

Store each as a separate *file* in a `data/routes_wgs/{route_name}.yaml` file.

```
# frame_id is not defined, as the robot does not require specifically a gnss frame
name: example_route
waypoints:
  start:        {lat: 50.00, lon: 14.00, mandatory: false}
  waypoint_1:   {lat: 50.01, lon: 14.01}
  waypoint_2:   {lat: 50.02, lon: 14.02}
  checkpoint_a: {lat: 50.03, lon: 14.03, max_linear_vel: 1.2, max_angular_vel: 1.2, lookahead_distance: 1.5}
  waypoint_3:   {lat: 50.04, lon: 14.04}
  waypoint_4:   {lat: 50.05, lon: 14.05}
  ...
```

### Recording waypoints and routes in RViz

The `commander_operator_rviz` package provides a **Waypoint Recorder** panel and a **WaypointClick** tool, the `waypoint_recorder` node of this package writes the clicked points into the files above.

```
ros2 launch commander_operator waypoint_recorder.launch.py data_dir:=<path to the package source>/data
```

Parameters of `waypoint_recorder`:

- `map_frame` (default `map`): frame local waypoints and routes are saved in, clicked points are transformed into it. `waypoints.yaml` must have the same `frame_id`.
- `ecef_frame` (default `earth`): ECEF frame used for WGS points (`clicked frame -> ecef_frame` by TF, then ECEF -> WGS by pyproj). The height is dropped, as when loading.
- `data_dir` (default: installed `share/commander_operator/data`): where the files are written. With `--symlink-install` editing the installed files changes the source ones, but new route files would only end up in `install/`, so point this to the source `data` folder. Use the same `data_dir` for `commander_operator`, the recorder asks it to reload (`commander_operator/reload`) after every save.
- `tf_timeout` (default `1.0` s).

In RViz:

1. *Panels -> Add New Panel -> commander_operator_rviz/WaypointRecorder*, optionally add a *MarkerArray* display on `/waypoint_recorder/pending_markers` to see the clicked points.
2. Choose *Waypoint* or *Route* and *Local* or *WGS*, press *Record* (this activates the WaypointClick tool, shortcut `w`) and click on the ground plane (z = 0 of the fixed frame).
   - *Waypoint*: a click places the point (a new click moves it), fill in the name and the parameters, then *Save*.
   - *Route*: every click appends a point with the name and parameters currently in the panel; an empty name gives `waypoint_N` (N = position in the route). Fill in the route name and *Save*. *Undo last* / *Clear* edit the pending points.
3. Existing waypoints / route files are replaced only with *Overwrite existing* checked.

New waypoints are appended to the files as a line, keeping the existing formatting and comments. Overwriting a waypoint regenerates the file and keeps only the comment block at its top.
