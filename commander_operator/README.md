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