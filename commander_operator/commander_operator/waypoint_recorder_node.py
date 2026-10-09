import os

from ament_index_python.packages import get_package_share_directory
from commander_operator.waypoints import (
    ECEF_TO_WGS, is_valid_name, LOCAL_DECIMALS, make_entry, save_route, save_waypoint,
    WaypointFileError, WGS_DECIMALS)
from commander_operator_interfaces.srv import SaveWaypoints
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
import tf2_geometry_msgs
import tf2_ros


class WaypointRecorder(Node):
    """
    Save waypoints and routes (e.g. clicked in rviz) into the commander_operator data files.

    Local points are transformed into `map_frame`, WGS points are transformed into
    `ecef_frame` and converted to latitude/longitude with pyproj.
    """

    def __init__(self):
        super().__init__('waypoint_recorder')

        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.ecef_frame = self.declare_parameter('ecef_frame', 'earth').value
        self.tf_timeout = self.declare_parameter('tf_timeout', 1.0).value

        # Note: to have the recorded points in the source tree, point this to the
        # data folder of the package source instead of the installed one.
        default_data_dir = os.path.join(
            get_package_share_directory('commander_operator'), 'data')
        self.data_dir = self.declare_parameter('data_dir', default_data_dir).value

        self.tf_buffer = tf2_ros.Buffer()
        # The listener uses a reentrant callback group, so with a multi threaded executor
        # (see main) the service callback can wait for transforms.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.save_srv = self.create_service(
            SaveWaypoints, '~/save_waypoints', self.save_waypoints_cb)
        # commander_operator's ~/reload, so that it knows about the saved goals right away.
        self.reload_client = self.create_client(Trigger, 'reload_waypoints')

    def save_waypoints_cb(self, request, response):
        try:
            response.file_path = self.save(request)
        except (WaypointFileError, tf2_ros.TransformException, OSError) as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Saving failed: {e}')
            return response

        what = (f"route '{request.route_name}' ({len(request.goals)} waypoints)"
                if request.route_name else f"waypoint '{request.names[0]}'")
        response.success = True
        response.message = f"Saved {what} to '{response.file_path}'."
        self.get_logger().info(response.message)

        if self.reload_client.service_is_ready():
            self.reload_client.call_async(Trigger.Request())
        else:
            response.message += ' commander_operator not running, not reloaded.'

        return response

    def save(self, request):
        """Validate and write the request, return the path of the written file."""
        if not request.goals:
            raise WaypointFileError('No waypoints to save.')
        if len(request.names) != len(request.goals):
            raise WaypointFileError(
                f'Got {len(request.names)} names for {len(request.goals)} waypoints.')
        for name in request.names:
            if not is_valid_name(name):
                raise WaypointFileError(
                    f"Invalid waypoint name '{name}' (use letters, digits, '_' and '-', "
                    'starting with a letter or "_").')
        if len(set(request.names)) != len(request.names):
            raise WaypointFileError('Waypoint names within a route must be unique.')

        route = bool(request.route_name)
        if route and not is_valid_name(request.route_name):
            raise WaypointFileError(f"Invalid route name '{request.route_name}'.")
        if not route and len(request.goals) != 1:
            raise WaypointFileError('Without a route name exactly one waypoint is saved.')

        entries = [
            (name, make_entry(goal, self.coordinates(goal, request.wgs), route))
            for name, goal in zip(request.names, request.goals)]
        frame_id = None if request.wgs else self.map_frame

        if route:
            folder = os.path.join(self.data_dir, 'routes_wgs' if request.wgs else 'routes')
            return save_route(folder, request.route_name, entries, frame_id, request.overwrite)

        path = os.path.join(
            self.data_dir, 'waypoints_wgs.yaml' if request.wgs else 'waypoints.yaml')
        name, entry = entries[0]
        save_waypoint(path, name, entry, frame_id, request.overwrite)
        return path

    def coordinates(self, goal, wgs):
        """Return the position of an OperatorGoal as {'x', 'y'} or {'lat', 'lon'}."""
        if wgs:
            p = self.transform(goal.goal, self.ecef_frame).pose.position
            lat, lon, _ = ECEF_TO_WGS.transform(p.x, p.y, p.z)
            return {'lat': round(lat, WGS_DECIMALS), 'lon': round(lon, WGS_DECIMALS)}

        p = self.transform(goal.goal, self.map_frame).pose.position
        return {'x': round(p.x, LOCAL_DECIMALS), 'y': round(p.y, LOCAL_DECIMALS)}

    def transform(self, pose_stamped, target_frame):
        source_frame = pose_stamped.header.frame_id
        if not source_frame:
            raise WaypointFileError('Waypoint has no frame_id.')
        if source_frame == target_frame:
            return pose_stamped
        # Latest transform: the points are static and clicked at an arbitrary time.
        transform = self.tf_buffer.lookup_transform(
            target_frame, source_frame, Time(), timeout=Duration(seconds=self.tf_timeout))
        return tf2_geometry_msgs.do_transform_pose_stamped(pose_stamped, transform)


def main():
    # Explicit init/shutdown: rclpy.init() is not a context manager before Kilted (e.g. Jazzy).
    rclpy.init()
    waypoint_recorder = None
    try:
        waypoint_recorder = WaypointRecorder()
        rclpy.spin(waypoint_recorder, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if waypoint_recorder is not None:
            waypoint_recorder.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
