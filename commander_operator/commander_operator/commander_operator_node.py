import os

from ament_index_python.packages import get_package_share_directory
from commander_operator.waypoints import load_routes, load_waypoints
from commander_operator_interfaces.msg import OperatorRequest
from crl_commander_interfaces.msg import OperatorGoal, OperatorGoalArray
from crl_commander_interfaces.srv import SwitchMode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger
# import tf2_ros


class CommanderOperator(Node):

    def __init__(self):
        super().__init__('commander_operator')

        self.ecef_frame = self.declare_parameter('ecef_frame', 'earth').value

        default_data_dir = os.path.join(
            get_package_share_directory('commander_operator'), 'data')
        self.data_dir = self.declare_parameter('data_dir', default_data_dir).value

        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # reliable qos by default
        self.commander_switch_mode_client = self.create_client(SwitchMode, 'switch_mode')

        # not necessary since we check when sending the request down the stream...
        # while not self.commander_switch_mode_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().warning(
        #         "'switch_mode' service of commander not available yet, "
        #         'start the commander node, waiting...')

        sub_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        pub_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_sub = self.create_subscription(
            OperatorRequest, 'goal', self.operator_request_cb, qos_profile=sub_qos)

        self.operator_goal_pub = self.create_publisher(
            OperatorGoal, 'operator_goal', qos_profile=pub_qos)
        self.operator_sequence_pub = self.create_publisher(
            OperatorGoalArray, 'operator_sequence', qos_profile=pub_qos)

        self.load_all()

        # Lets other nodes (e.g. waypoint_recorder) make newly saved goals available.
        self.reload_srv = self.create_service(Trigger, '~/reload', self.reload_cb)

        self.request_id = 0

    def load_all(self):
        logger = self.get_logger()
        self.waypoints = load_waypoints(
            os.path.join(self.data_dir, 'waypoints.yaml'), logger, None)
        self.waypoints_wgs = load_waypoints(
            os.path.join(self.data_dir, 'waypoints_wgs.yaml'), logger, self.ecef_frame)
        self.routes = load_routes(
            os.path.join(self.data_dir, 'routes'), logger, None)
        self.routes_wgs = load_routes(
            os.path.join(self.data_dir, 'routes_wgs'), logger, self.ecef_frame)

    def reload_cb(self, request, response):
        self.load_all()
        response.success = True
        response.message = (
            f'Loaded {len(self.waypoints)} waypoints, {len(self.waypoints_wgs)} wgs waypoints, '
            f'{len(self.routes)} routes and {len(self.routes_wgs)} wgs routes.')
        self.get_logger().info(response.message)
        return response

    def get_goal_object_and_request(self, goal, route, wgs):
        goal_object = None
        request = SwitchMode.Request()
        if route:
            if wgs:
                if goal in self.routes_wgs:
                    goal_object = self.routes_wgs[goal]
                    request.mode = SwitchMode.Request.MODE_SEQUENCE
            else:
                if goal in self.routes:
                    goal_object = self.routes[goal]
                    request.mode = SwitchMode.Request.MODE_SEQUENCE
        else:
            if wgs:
                if goal in self.waypoints_wgs:
                    goal_object = self.waypoints_wgs[goal]
                    request.mode = SwitchMode.Request.MODE_GOTO
            else:
                if goal in self.waypoints:
                    goal_object = self.waypoints[goal]
                    request.mode = SwitchMode.Request.MODE_GOTO

        return goal_object, request

    def operator_request_cb(self, msg):
        wgs = msg.wgs
        route = msg.route
        goal = msg.goal

        goal_object, request = self.get_goal_object_and_request(goal, route, wgs)

        if goal_object is None:
            self.get_logger().error(
                f"There is no {'route' if route else 'waypoint'} in "
                f"{'wgs' if wgs else 'local frame'} named '{goal}'. Cannot proceed.")
            return

        if not self.commander_switch_mode_client.service_is_ready():
            self.get_logger().error(
                "'switch_mode' service of commander not available, dropping request. "
                'Check if commander has been started.')
            return

        # Publish goal first
        if route:
            self.operator_sequence_pub.publish(goal_object.get_route())
        else:
            self.operator_goal_pub.publish(goal_object.goal)

        # Send switch_mode request second
        self.request_id += 1
        req_id = self.request_id
        future = self.commander_switch_mode_client.call_async(request)
        future.add_done_callback(
            lambda f: self.switch_mode_done(f, req_id, request.mode, msg.goal))

    def switch_mode_done(self, future, req_id, mode, goal_name):
        if req_id != self.request_id:
            return  # a newer operator request superseded this one
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"switch_mode '{mode}' call failed: {e}")
            return
        if res.success:
            self.get_logger().info(f"'{goal_name}' sent, commander: {res.message}")
        else:
            self.get_logger().error(f"Commander rejected mode '{mode}': {res.message}")


def main():
    try:
        with rclpy.init():
            commander_operator = CommanderOperator()
            rclpy.spin(commander_operator)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
