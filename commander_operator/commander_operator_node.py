from ament_index_python.packages import get_package_share_directory
from commander_operator_interfaces.msg import OperatorRequest
from crl_commander_interfaces.msg import OperatorGoal, OperatorGoalArray
from crl_commander_interfaces.srv import SwitchMode
import glob
import os
import pyproj
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
# import tf2_ros
import yaml

WGS_TO_ECEF = pyproj.Transformer.from_crs("EPSG:4326","EPSG:4978")

class Waypoint:
    def __init__(self, name = None):
        # Only 'mandatory' has a reasonable default value, others need flags to indicate whether they are being set or not.
        self.use_max_linear_vel =     False
        self.use_max_angular_vel =    False
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
        if ('max_linear_vel' in waypoint_dict):
            self.max_linear_vel =     float(waypoint_dict['max_linear_vel'])
            self.use_max_linear_vel = True

        if ('max_angular_vel' in waypoint_dict):
            self.max_angular_vel =    float(waypoint_dict['max_angular_vel'])
            self.use_max_angular_vel = True
            
        if ('lookahead_distance' in waypoint_dict):
            self.lookahead_distance = float(waypoint_dict['lookahead_distance'])
            self.use_lookahead_distance = True

        if ('mandatory' in waypoint_dict):
            self.mandatory =          waypoint_dict['mandatory']

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
            return False, "Did not find neither lat,lon nor x,y."

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

        return True, "Success."

class Route:
    def __init__(self):
        self.waypoints = []

    def get_route(self, selected_waypoints=None):
        route = OperatorGoalArray()
        if selected_waypoints is not None:
            raise NotImplementedError("to be done")
        else:
            route.goals = list(map(lambda w : w.goal, self.waypoints))

        return route

    def from_dict(self, waypoint_dict, frame_id):
        failed_waypoints = []
        failed_waypoints_msgs = []
        
        for i,name in enumerate(waypoint_dict['waypoints'].keys()):
            waypoint = waypoint_dict['waypoints'][name]
            w = Waypoint(name)
            success,msg = w.from_dict(waypoint, frame_id)
            if not success:
                failed_waypoints.append(i+1)
                failed_waypoints_msgs.append(msg)
                continue

            self.waypoints.append(w)

        if len(failed_waypoints):
            return False, f"Failed to load waypoints number: {failed_waypoints}. For following reasons: {failed_waypoints_msgs}."

        return True, "Success."

class CommanderOperator(Node):
    def __init__(self):
        super().__init__("commander_operator")
        
        self.ecef_frame = self.declare_parameter("ecef_frame",'earth').value

        default_data_dir = os.path.join(get_package_share_directory('commander_operator'), 'data')
        self.data_dir = self.declare_parameter('data_dir', default_data_dir).value

        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.commander_switch_mode_client = self.create_client(SwitchMode, "switch_mode") # reliable qos by default

        # not necessary since we check when sending the request down the stream...
        # while not self.commander_switch_mode_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().warning("'switch_mode' service of commander not available yet, start the commander node, waiting...")

        sub_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        pub_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.goal_sub = self.create_subscription(OperatorRequest, "goal", self.operator_request_cb, qos_profile=sub_qos)

        self.operator_goal_pub = self.create_publisher(OperatorGoal, "operator_goal", qos_profile=pub_qos)
        self.operator_sequence_pub = self.create_publisher(OperatorGoalArray, "operator_sequence", qos_profile=pub_qos)
        
        self.waypoints =     self.load_waypoints(os.path.join(self.data_dir, "waypoints.yaml"), None)
        self.waypoints_wgs = self.load_waypoints(os.path.join(self.data_dir, "waypoints_wgs.yaml"), self.ecef_frame)
        self.routes =        self.load_routes(os.path.join(self.data_dir, "routes"), None) 
        self.routes_wgs =    self.load_routes(os.path.join(self.data_dir, "routes_wgs"), self.ecef_frame)

        self.request_id = 0

    def load_waypoints(self, file_path, default_frame_id=None):
        waypoints = dict()

        if not os.path.exists(file_path):
            self.get_logger().warning(f"File {file_path} does not exist. Cannot load waypoints from it.")

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
                    self.get_logger().error("Frame_id not provided for non-wgs waypoints! Cannot proceed!")
                    continue
                    
                w = Waypoint(waypoint_name)
                success,msg = w.from_dict(waypoint_dict, frame_id)

                if success:
                    if waypoint_name in waypoints.keys():
                        self.get_logger().warning(f"Detected duplicate waypoint '{waypoint_name}' in '{file_path}', overwriting the previous one!")
                    waypoints[waypoint_name] = w
                else:
                    self.get_logger().error("Failed to load waypoint: '"+msg+"'")

        return waypoints

    def load_routes(self, folder_path, default_frame_id=None):
        routes = dict()
        route_files = glob.glob(os.path.join(folder_path,"*.yaml"))

        if not os.path.exists(folder_path):
            self.get_logger().warning(f"Folder {folder_path} does not exist. Cannot load routes from it.")

        for route_file in route_files:
            with open(route_file, 'r') as f:
                waypoints_yaml = yaml.safe_load(f)

                frame_id = None
                if 'frame_id' in waypoints_yaml:
                    frame_id = waypoints_yaml['frame_id']
                elif default_frame_id is not None:
                    frame_id = default_frame_id
                else:
                    self.get_logger().error("Frame_id not provided for non-wgs routes! Cannot proceed!")
                    continue
                        
                route = Route()
                success,msg = route.from_dict(waypoints_yaml, frame_id)
                
                if success:
                    name = waypoints_yaml['name']
                    if name in routes.keys():
                        self.get_logger().warning(f"Detected duplicate route '{name}' in '{folder_path}', overwriting the previous one!")
                    routes[name] = route 
                else:
                    self.get_logger().error("Failed to load route: '"+msg+"'")

        return routes

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

        return goal_object,request

    def operator_request_cb(self, msg):
        wgs = msg.wgs
        route = msg.route
        goal = msg.goal

        goal_object,request = self.get_goal_object_and_request(goal, route, wgs)

        if goal_object is None:
            self.get_logger().error(f"There is no {'route' if route else 'waypoint'} in {'wgs' if wgs else 'local frame'} named '{goal}'. Cannot proceed.")
            return

        if not self.commander_switch_mode_client.service_is_ready():
            self.get_logger().error("'switch_mode' service of commander not available, dropping request. Check if commander has been started.")
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
        future.add_done_callback(lambda f: self.switch_mode_done(f, req_id, request.mode, msg.goal))

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
