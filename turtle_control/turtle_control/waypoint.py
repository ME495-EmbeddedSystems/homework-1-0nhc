import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoints
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import numpy as np
import copy

MOVING = 0
STOPPED = 1
    

class DWAPlanner:
    def __init__(self) -> None:
        # Robot's dynamic parameters
        self._MIN_VEL = 0.0  # Maximum linear velocity [m/s]
        self._MAX_VEL = 3.0  # Maximum linear velocity [m/s]
        self._MIN_OMEGA = -7.0  # Maximum angular velocity [rad/s]
        self._MAX_OMEGA = 7.0  # Maximum angular velocity [rad/s]

        # Sampling parameters
        self._DT = 0.01  # Time step for simulating trajectories
        self._PREDICTION_TIME = 0.3  # Predict over the next 2 seconds
        self._NUM_SAMPLES = 15

        # Cost function weights
        self._ALPHA = 1.0  # Weight for goal progress
        self._GAMMA = 1.0  # Weight for velocity
        
    
    
    def dwa_control(self, current_state, goal):
        x = current_state[0]
        y = current_state[1]
        theta = current_state[2]
        xg = goal[0]
        yg = goal[1]
        
        best_v, best_w = 0, 0
        best_cost = float('inf')
        
        # Define the dynamic window based on current state and constraints
        vel_samples = np.linspace(self._MIN_VEL, self._MAX_VEL, self._NUM_SAMPLES)
        omega_samples = np.linspace(self._MIN_OMEGA, self._MAX_OMEGA, self._NUM_SAMPLES)

        # Loop through all velocity samples
        for v in vel_samples:
            for w in omega_samples:
                # Predict the future trajectory
                predicted_x, predicted_y, predicted_theta = self._predict_trajectory_ending(x, y, theta, v, w)

                # Calculate costs for this trajectory
                cost_goal = self._goal_cost(predicted_x, predicted_y, xg, yg)
                cost_velocity = self._velocity_cost(v)

                total_cost = self._ALPHA * cost_goal + self._GAMMA * cost_velocity

                if total_cost < best_cost:
                    best_cost = total_cost
                    best_v, best_w = v, w

        return [best_v, best_w]
    
    
    def _predict_trajectory_ending(self, x, y, theta, v, w):
        pred_x, pred_y, pred_theta = x, y, theta
        for _ in range(int(self._PREDICTION_TIME / self._DT)):
            pred_x += v * np.cos(pred_theta) * self._DT
            pred_y += v * np.sin(pred_theta) * self._DT
            pred_theta += w * self._DT
        return pred_x, pred_y, pred_theta
    
    
    def _goal_cost(self, px, py, xg, yg):
        # We want the smallest Euclidean distance
        return np.sqrt((px - xg) ** 2 + (py - yg) ** 2)
    
    
    def _velocity_cost(self, v):
        # We want the fastest speed
        return 1.0 / (v + 1e-3)
    
    
class WaypointNode(Node):

    def __init__(self):
        super().__init__('waypoint_node')
        # Set logger level to DEBUG
        # self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Declare ROS 2 parameters
        # Declare timer frequency parameter, default to 90 Hz
        self.declare_parameter('frequency', 90.0)
        self._timer_frequency = self.get_parameter("frequency").get_parameter_value().double_value
        # Declare robot name, default to turtle1
        self.declare_parameter('robot_name', 'turtle1')
        self._robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        # Remove '/'
        while(self._robot_name[0] == '/'):
            self._robot_name = self._robot_name[1:]
            self.get_logger().info(self._robot_name)
        # Declare goal tolerance parameter, default to 0.1
        self.declare_parameter('tolerance', 0.1)
        self._tolerance = self.get_parameter("tolerance").get_parameter_value().double_value
        
        # Enable dynamic reconfiguration for parameters
        self.add_on_set_parameters_callback(self._parameter_callback)
        
        # Setup ROS 2 timer
        self._timer = self.create_timer(1.0/self._timer_frequency, self._timer_callback)
        
        # Setup ROS 2 Service
        self._toggle_servive = self.create_service(Empty, 'toggle', self._toggle_callback)
        self._load_servive = self.create_service(Waypoints, 'load', self._load_callback)
        
        # Setup ROS 2 Client
        # Reset turtlesim client
        self._reset_callback_group = MutuallyExclusiveCallbackGroup()
        self._reset_turtlesim_client = self.create_client(Empty, 'reset', callback_group=self._reset_callback_group)
        while not self._reset_turtlesim_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for turtlesim's service: /reset")
        self._reset_turtlesim_request = Empty.Request()
        
        # Teleport turtle absolute client
        self._teleport_absolute_callback_group = MutuallyExclusiveCallbackGroup()
        self._teleport_absolute_client = self.create_client(TeleportAbsolute, self._robot_name+'/teleport_absolute', callback_group=self._teleport_absolute_callback_group)
        while not self._teleport_absolute_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for turtlesim's service: "+self._robot_name+"/teleport_absolute")
        self._teleport_absolute_request = TeleportAbsolute.Request()
        
        # Turtlesim set pen client
        self._set_pen_callback_group = MutuallyExclusiveCallbackGroup()
        self._set_pen_client = self.create_client(SetPen, self._robot_name+'/set_pen', callback_group=self._set_pen_callback_group)
        while not self._set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for turtlesim's service: "+self._robot_name+"/set_pen")
        self._set_pen_request = SetPen.Request()
        
        # Setup subscriber for turtle's pose
        self._turtle_pose_subscriber = self.create_subscription(Pose, self._robot_name+'/pose', self._turtle_pose_callback, 10)
        self._turtle_pose_subscriber  # prevent unused variable warning
        
        # Setup publisher to control the turtle's movements
        self._turtle_cmd_publisher = self.create_publisher(Twist, self._robot_name+'/cmd_vel', 10)
        self._turtle_cmd = Twist()
        
        # Initialize node state
        self._state = STOPPED
        # Initialize waypoints
        self._waypoints = []
        self._following_points = []
        # Initialize turtle pose
        self._turtle_pose = Pose()
        
        # Initialize motion controller
        self._dwa_controller = DWAPlanner()
        

    def _timer_callback(self):
        if(self._state == MOVING):
            self.get_logger().debug('Issuing Command!')
            # Make sure waypoints are loaded
            if(len(self._waypoints) == 0):
                self.get_logger().error('No waypoints loaded. Load them with the "load" service.')
            else:
                # Follow the waypoints
                # Check if there are points to follow
                if(len(self._following_points) != 0):
                    # Check if turtle has reached the goal point
                    goal_point = self._following_points[0]
                    distance = np.sqrt((self._turtle_pose.x - goal_point[0])**2+(self._turtle_pose.y - goal_point[1])**2)
                    if(distance < self._tolerance):
                        self._following_points.pop(0)
                    else:
                        current_state = [self._turtle_pose.x, self._turtle_pose.y, self._turtle_pose.theta]
                        self._turtle_cmd.linear.x, self._turtle_cmd.angular.z = self._dwa_controller.dwa_control(current_state, goal_point)
                        self._turtle_cmd_publisher.publish(self._turtle_cmd)
                        # self.get_logger().info("best v: "+str(self._turtle_cmd.linear.x)+", best w: "+str(self._turtle_cmd.angular.z))
                else:
                    # Reload the waypoints
                    self._following_points = copy.deepcopy(self._waypoints)
                    self.get_logger().info('Turtle reached the end of waypoints.')
                
                
        elif(self._state == STOPPED):
            # Stop the turtle
            self._turtle_cmd.linear.x = float(0)
            self._turtle_cmd.angular.z = float(0)
            self._turtle_cmd_publisher.publish(self._turtle_cmd)
    
    
    async def _draw_an_X(self, x, y):
        # Set parameters for drawing the X
        X_length = 0.15
        self._set_pen_request.r = int(255)
        self._set_pen_request.g = int(0)
        self._set_pen_request.b = int(0)
        self._set_pen_request.width = int(2)
        
        # Turn off set pen service
        self._set_pen_request.off = True
        await self._set_pen_client.call_async(self._set_pen_request)
        
        # Go to the center point of the X first
        self._teleport_absolute_request.x = x
        self._teleport_absolute_request.y = y
        await self._teleport_absolute_client.call_async(self._teleport_absolute_request)
        
        # Turn on set pen service
        self._set_pen_request.off = False
        await self._set_pen_client.call_async(self._set_pen_request)
        
        # Upper-right point of the X
        self._teleport_absolute_request.x = x + X_length
        self._teleport_absolute_request.y = y + X_length
        await self._teleport_absolute_client.call_async(self._teleport_absolute_request)
        
        # Lower-left point of the X
        self._teleport_absolute_request.x = x - X_length
        self._teleport_absolute_request.y = y - X_length
        await self._teleport_absolute_client.call_async(self._teleport_absolute_request)
        
        # Center point of the X
        self._teleport_absolute_request.x = x
        self._teleport_absolute_request.y = y
        await self._teleport_absolute_client.call_async(self._teleport_absolute_request)
        
        # Upper-left point of the X
        self._teleport_absolute_request.x = x - X_length
        self._teleport_absolute_request.y = y + X_length
        await self._teleport_absolute_client.call_async(self._teleport_absolute_request)
        
        # Lower-right point of the X
        self._teleport_absolute_request.x = x + X_length
        self._teleport_absolute_request.y = y - X_length
        await self._teleport_absolute_client.call_async(self._teleport_absolute_request)
        
        # Turn off set pen service
        self._set_pen_request.off = True
        await self._set_pen_client.call_async(self._set_pen_request)
            
        # self.get_logger().info(str(self._teleport_absolute_request))
    
    
    async def _load_callback(self, request, response):
        # Initialize distance response
        distance = 0.0
        
        # Read waypoints from request
        for idx, point in enumerate(request.waypoints):
            # Draw an X
            await self._draw_an_X(point.x, point.y)
            # Store the point
            self._waypoints.append([point.x, point.y])
            self._following_points.append([point.x, point.y])
            # Calc the distance
            if(idx != len(request.waypoints)-1):
                d = np.sqrt((point.x - request.waypoints[idx+1].x)**2+(point.y - request.waypoints[idx+1].y)**2)
                distance += d
        
        # Go back to zero position
        self._teleport_absolute_request.x = 5.544445
        self._teleport_absolute_request.y = 5.544445
        await self._teleport_absolute_client.call_async(self._teleport_absolute_request)
        
        # Turn on set pen service
        self._set_pen_request.r = int(0)
        self._set_pen_request.g = int(255)
        self._set_pen_request.b = int(0)
        self._set_pen_request.width = int(3)
        self._set_pen_request.off = False
        await self._set_pen_client.call_async(self._set_pen_request)
        
        # Make sure the state is STOPPED
        self._state = STOPPED
        
        # Wrap the response
        response.distance = float(distance)
            
        return response
    
    
    def _toggle_callback(self, request, response):
        if(self._state == STOPPED):
            self._state = MOVING
        elif(self._state == MOVING):
            # When switching from MOVING to STOPPED the node INFO logs "Stopping." one time.
            self.get_logger().info("Stopping.")
            self._state = STOPPED
        else:
            self.get_logger().warn("Unknown state: "+str(self._state))
            
        return response
    
    
    def _turtle_pose_callback(self, msg):
        self._turtle_pose = msg
    
    
    def _parameter_callback(self, params):
        for param in params:
            self.get_logger().info('PARAMETER NEW VALUE, '+str(param.name)+': '+str(param.value))
        
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    waypoint_node = WaypointNode()
    rclpy.spin(waypoint_node)
    waypoint_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()