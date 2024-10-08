"""The waypoint ROS 2 node for hopmework-1, along with a DWA planner class and an odometry class.

The waypoint node communicates through several ROS 2 protocols:

PUBLISHERS:
  + turtle1/cmd_vel (geometry_msgs.msg.Twist) - The velocity command to control the turtle
  + loop_metrics (turtle_interfaces.msg.ErrorMetric) - Indicating the turtle's trajectory information.

SUBSCRIBERS:
  + turtle1/pose (turtlesim.msg.Pose) - The pose state of the turtle

SERVICES:
  + toggle (std_srvs.srv.Empty) - To toggle the turtle's state (STOPPED/MOVING)
  + load (turtle_interfaces.srv.Waypoints) - To load waypoints for the turtle to follow

CLIENTS:
  + reset (std_srvs.srv.Empty) - To reset the turtlesim simulator
  + turtle1/teleport_absolute (turtlesim.srv.TeleportAbsolute) - To teleport the turtle to specific 
    positions
  + turtle1/set_pen (turtlesim.srv.SetPen) - To control the pen in turtlesim
  
PARAMETERS:
  + frequency (double) - Timer frequency to control the turtle's movements
  + tolerance (double) - The tolerance to check whether the turtle has arrived at the waypoint
  
  Parameters below are related to other features I added for fun.
  + robot_name (string) - The robot name (default to "turtle1" in this project). I do this because
    I think in the future this project can be expanded to a multi-robot system.
  + rainbow_frequency (double) - Timer frequency to control the pen color in turtlesim. I do this 
    because I heard another guy from the last cohort did this, and I think it's interesting.
  + rainbow_increment_speed (double) - The speed of the rainbow
  
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoints
from turtle_interfaces.msg import ErrorMetric
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import numpy as np
import copy
import random 

# Predifined states
MOVING = 0
STOPPED = 1
    

############################### Begin_Citation [1] ###############################
class DWAPlanner:
    """DWA Planner for the turtle's motion control.

    I use DWA planner to control the turtle to follow the waypoints.
    This class is accomplished with the help of ChatGPT 4o. 
    Related conversaiton link is in the citations.txt

    Attributes:
        _MIN_VEL: Minimum linear velocity [m/s]
        _MAX_VEL: Maximum linear velocity [m/s]
        _MIN_OMEGA: Minimum angular velocity [rad/s]
        _MAX_OMEGA: Maximum angular velocity [rad/s]
        _DT: Time step for simulating trajectories
        _PREDICTION_TIME: Predict over the next _PREDICTION_TIME seconds
        _NUM_SAMPLES: Number of velocity samples
        _ALPHA: Weight for goal progress
        _GAMMA: Weight for velocity
        
    """
    def __init__(self) -> None:
        """Initializes the DWA Planner parameters
        
        Args:
            None
        """
        # Robot's dynamic parameters
        self._MIN_VEL = 0.0  # Minimum linear velocity [m/s]
        self._MAX_VEL = 3.0  # Maximum linear velocity [m/s]
        self._MIN_OMEGA = -8.0  # Minimum angular velocity [rad/s]
        self._MAX_OMEGA = 8.0  # Maximum angular velocity [rad/s]

        # Sampling parameters
        self._DT = 0.01  # Time step for simulating trajectories
        self._PREDICTION_TIME = 0.2  # Predict over the next _PREDICTION_TIME seconds
        self._NUM_SAMPLES = 11 # Number of velocity samples

        # Cost function weights
        self._ALPHA = 1.0  # Weight for goal progress
        self._GAMMA = 1.0  # Weight for velocity
        
    
    def dwa_control(self, current_state, goal):
        """Give control input for the turtle's movements

        Given goal point position and turtle's current states, this function
        applies DWA to get the optimal velocity input for the turtle.

        Args:
            current_state: A list consisting of [x, y, theta], indicating the turtle's current state.
            goal: A list consisting of [x, y], indicating the turtle's goal position.

        Returns:
            A list consisting of [best_v, best_w], indicating the controller's optimal input for the 
            turtle. Here, best_v is the optimal linear velocity along the X axis, and best_w is the 
            optimal angular velocity along the Z axis.
            
            Example:
            [1.0, 0.5]
            
            Returned values are float.

        Raises:
            None
        """
        # Load args for programming convenience
        x = current_state[0]
        y = current_state[1]
        theta = current_state[2]
        xg = goal[0]
        yg = goal[1]
        
        # Initialize controller variables
        best_v, best_w = 0, 0
        minimum_cost = float('inf')
        
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
                
                # Get the optimal input
                if total_cost < minimum_cost:
                    minimum_cost = total_cost
                    best_v, best_w = v, w

        return [best_v, best_w]
    
    
    def _predict_trajectory_ending(self, x, y, theta, v, w): 
        """Form current states and sampled velocities, compute the turtle's final state after 
        self._PREDICTION_TIME seconds.

        Given turtle's current states and sampled velocities, this function computes the turtle's 
        final state after self._PREDICTION_TIME seconds with a differential type mobile robot model.

        Args:
            x: The turtle's current x position
            y: The turtle's current y position
            theta: The turtle's current theta
            v: The sampled linear velocity along the X axis
            w: The sampled angular velocity along the Z axis

        Returns:
            The turtle's final state after self._PREDICTION_TIME seconds with sampled velocities.
            
            Example:
            4.1, 5.3, 0.3
            
            Returned values are float.

        Raises:
            None
        """
        pred_x, pred_y, pred_theta = x, y, theta
        for _ in range(int(self._PREDICTION_TIME / self._DT)):
            pred_x += v * np.cos(pred_theta) * self._DT
            pred_y += v * np.sin(pred_theta) * self._DT
            pred_theta += w * self._DT
        return pred_x, pred_y, pred_theta
    
    
    def _goal_cost(self, px, py, xg, yg):
        """Compute the goal cost of a trajectory derived form sampled velocities.

        Given the end point position of a sample trajectory based on a sample velocities and the 
        waypoint goal position, compute the goal cost of this trajectory based on Euclidean distance.
        The smaller distance , the lower cost.

        Args:
            px: The end point position x of a sample trajectory based on a sample velocities
            py: The end point position y of a sample trajectory based on a sample velocities
            xg: The waypoint goal position x
            yg: The waypoint goal position y

        Returns:
            The goal cost. We want the smallest Euclidean distance. 
            The smaller distance , the lower cost.
            
            Example:
            0.3
            
            Returned value is float.

        Raises:
            None
        """
        # We want the smallest Euclidean distance
        return np.sqrt((px - xg) ** 2 + (py - yg) ** 2)
    
    
    def _velocity_cost(self, v):
        """Compute the velocity cost of the sampled linear velocity.

        Given a sampled linear velocity, this funciton computes the velocty cost based on it.
        The smaller velocity , the higher cost.

        Args:
            v: the sampled linear velocity

        Returns:
            The velocity cost. We want the fastest speed.
            The smaller velocity , the higher cost.
            
            Example:
            0.3
            
            Returned value is float.

        Raises:
            None
        """
        # We want the fastest speed
        return 1.0 / (v + 1e-3)
############################### End_Citation [1]  ###############################


class DifferentialOdometry:
    """Differential Odometry for the turtle's travelled distance.

    This is a typical odometry for differential type mobile robots.

    Attributes:
        _x: The turtle's x position state [m]
        _y: The turtle's y position state [m]
        _theta: The turtle's theta orientation state [rad]
        _time: Travelled time [s]
        _mode: The waypoint node's mode [MOVING/STOPPED]
        _travelled_distance: The distance the turtle has travelled [m]
    """
    def __init__(self, initial_x=0.0, initial_y=0.0, initial_theta=0.0, initial_time=0.0) -> None:
        """Initializes the Differential Odometry parameters
        
        Args:
            initial_x: The initial x position state of the turtle.
            initial_y: The initial y position state of the turtle.
            initial_theta: The initial theta orientation state of the turtle.
            initial_time: The initial travelled time.
        """
        # Odometry states
        self._x =  initial_x # The turtle's x position state [m]
        self._y =  initial_y # The turtle's y position state [m]
        self._theta =  initial_theta # The turtle's theta orientation state [rad]
        self._time = initial_time # Travelled time [s]
        self._mode = STOPPED # The waypoint node's mode [MOVING/STOPPED]
        self._travelled_distance = 0.0 # The distance the turtle has travelled [m]
    
    
    def update_odom(self, v, w, t):
        """Update odometry with new states

        Given turtle's velocity states and current time, update the odometry states.

        Args:
            v: Current linear velocity along the X axis
            w: Current angular velocity along the Z axis
            t: Current time in seconds

        Returns:
            None

        Raises:
            None
        """
        if(self._mode == STOPPED):
            # Start the odometry
            self._mode = MOVING
            self._time = t
        elif(self._mode == MOVING):
            # Begin computing
            dt = t - self._time
            dx = v*np.cos(self._theta)*dt
            dy = v*np.sin(self._theta)*dt
            dtheta = w**dt
            ddistance = np.sqrt((dx)**2+(dy)**2)
            
            self._x += dx
            self._y += dy
            self._theta += dtheta
            self._travelled_distance += ddistance
            
            self._time = t
    
    
    def clear_states(self):
        """Clear odometry states

        In this project, when a loop is completed, we have calculate the travelled distance again.
        Thus, we need a function to clear the odometry's states.

        Args:
            None

        Returns:
            None

        Raises:
            None
        """
        self._x =  0.0
        self._y =  0.0
        self._theta =  0.0
        self._time = 0.0
        self._mode = STOPPED
        self._travelled_distance = 0.0
    
    
    def get_travelled_distancce(self):
        """Return travelled distance

        When a loop circle is completed, we need to get the travelled distance. This funciton
        returns the self._travelled_distance value.

        Args:
            None

        Returns:
            self._travelled_distance
            
            Example:
            31.352
            
            Returned value is float.

        Raises:
            None
        """
        return self._travelled_distance
    
    
class WaypointNode(Node):

    def __init__(self):
        super().__init__('waypoint_node')
        # Set logger level to DEBUG
        # self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Declare ROS 2 parameters
        # Declare timer frequency parameter, default to 90 Hz
        self.declare_parameter('frequency', 90.0)
        self._timer_frequency = self.get_parameter("frequency").get_parameter_value().double_value
        # Declare rainbow frequency parameter, default to 10 Hz
        self.declare_parameter('rainbow_frequency', 10.0)
        self._rainbow_frequency = self.get_parameter("rainbow_frequency").get_parameter_value().double_value
        # Declare robot name, default to turtle1
        self.declare_parameter('robot_name', 'turtle1')
        self._robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        # Remove '/'
        while(self._robot_name[0] == '/'):
            self._robot_name = self._robot_name[1:]
            # self.get_logger().info(self._robot_name)
        # Declare goal tolerance parameter, default to 0.1
        self.declare_parameter('tolerance', 0.1)
        self._tolerance = self.get_parameter("tolerance").get_parameter_value().double_value
        # Declare goal tolerance parameter, default to 1.0 (0.0 - 1.0)
        self.declare_parameter('rainbow_increment_speed', 1.0)
        self._rainbow_increment_speed = self.get_parameter("rainbow_increment_speed").get_parameter_value().double_value*float(50)
        # self.get_logger().info(str(self._rainbow_increment_speed ))
        
        # Enable dynamic reconfiguration for parameters
        self.add_on_set_parameters_callback(self._parameter_callback)
        
        # Setup ROS 2 timers
        self._timer = self.create_timer(1.0/self._timer_frequency, self._timer_callback)
        self._rainbow_timer = self.create_timer(1.0/self._rainbow_frequency, self._rainbow_timer_callback)
        
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
        
        # Setup publisher to fro loop metrics
        self._loop_metrics_publisher = self.create_publisher(ErrorMetric, '/loop_metrics', 10)
        self._loop_metrics = ErrorMetric()
        self._loop_metrics.complete_loops = int(0)
        self._loop_metrics.actual_distance = float(0)
        self._loop_metrics.error = float(0)
        
        # Initialize node state
        self._state = STOPPED
        # Initialize waypoints
        self._waypoints = []
        self._following_points = []
        self._waypoints_distance = 0.0
        # Initialize turtle pose
        self._turtle_pose = Pose()
        # Flag for start recording odometry
        self._following_the_first_goal = True
        # For generating rainbow rgb
        self._rainbow_seed = 0
        
        # Initialize motion controller
        self._dwa_controller = DWAPlanner()
        # Initialize odometry
        self._turtle_odometry = DifferentialOdometry()
        
        
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
                    # Control the turtle
                    if(distance < self._tolerance):
                        # Turtle has reached current goal, change the goal point
                        self._following_points.pop(0)
                        self._following_the_first_goal = False
                    else:
                        # Turtle hasn't reached the goal
                        # Compute error metrics
                        if (self._following_the_first_goal == False):
                            current_v = self._turtle_pose.linear_velocity
                            current_w = self._turtle_pose.angular_velocity
                            current_t = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]*1e-9
                            self._turtle_odometry.update_odom(current_v, current_w, current_t)
                        
                        # Control the turtle's movements
                        current_state = [self._turtle_pose.x, self._turtle_pose.y, self._turtle_pose.theta]
                        self._turtle_cmd.linear.x, self._turtle_cmd.angular.z = self._dwa_controller.dwa_control(current_state, goal_point)
                        self._turtle_cmd_publisher.publish(self._turtle_cmd)
                        # self.get_logger().info("best v: "+str(self._turtle_cmd.linear.x)+", best w: "+str(self._turtle_cmd.angular.z))
                        
                else:
                    # One cycle is achieved
                    
                    # Publish loop metrics
                    self._loop_metrics.complete_loops += int(1)
                    self._loop_metrics.actual_distance = float(self._turtle_odometry.get_travelled_distancce())
                    self._loop_metrics.error = float(self._loop_metrics.actual_distance - self._waypoints_distance)
                    self._loop_metrics_publisher.publish(self._loop_metrics)
                    
                    # Reset the following waypoints
                    self._following_points = copy.deepcopy(self._waypoints)
                    self.get_logger().info('Turtle reached the end of waypoints.')
                    
                    # Reset the turtle's odometry
                    self._turtle_odometry.clear_states()
                    self._following_the_first_goal = True
                    
                    # Stop the turtle
                    self._turtle_cmd.linear.x = float(0)
                    self._turtle_cmd.angular.z = float(0)
                    self._turtle_cmd_publisher.publish(self._turtle_cmd)
                
                
        elif(self._state == STOPPED):
            # Stop the turtle
            self._turtle_cmd.linear.x = float(0)
            self._turtle_cmd.angular.z = float(0)
            self._turtle_cmd_publisher.publish(self._turtle_cmd)
    

    async def _rainbow_timer_callback(self):
        if(self._state == MOVING):
            seed = float(self._rainbow_seed)/float(256*3)
            r = int(127*np.sin((2*np.pi)*seed) + 128) # 0 - 255
            g = int(127*np.sin((2*np.pi)*seed - 4*np.pi/3) + 128) # 0 - 255
            b = int(127*np.sin((2*np.pi)*seed - 2*np.pi/3) + 128) # 0 - 255
            # self.get_logger().info("r: "+str(r)+", g: "+str(g)+",, b: "+str(b))
            self._set_pen_request.r = r
            self._set_pen_request.g = g
            self._set_pen_request.b = b
            self._set_pen_request.width = int(5)
            self._set_pen_request.off = False
            await self._set_pen_client.call_async(self._set_pen_request)
            
            # Update the rainbow seed
            self._rainbow_seed += self._rainbow_increment_speed
            self._rainbow_seed = self._rainbow_seed%(256*3)
        
        
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
        # Reset error metrics
        self._loop_metrics.complete_loops = int(0)
        self._loop_metrics.actual_distance = float(0)
        self._loop_metrics.error = float(0)
        
        # Reset the turtle's odometry
        self._turtle_odometry.clear_states()
        
        # Read waypoints from request
        self._waypoints = []
        self._following_points = []
        self._waypoints_distance = 0.0
        for idx, point in enumerate(request.waypoints):
            # Draw an X
            await self._draw_an_X(point.x, point.y)
            # Store the point
            self._waypoints.append([point.x, point.y])
            self._following_points.append([point.x, point.y])
            # Calc the distance
            if(idx != len(request.waypoints)-1):
                d = np.sqrt((point.x - request.waypoints[idx+1].x)**2+(point.y - request.waypoints[idx+1].y)**2)
                self._waypoints_distance += d
        
        # Go back to zero position
        self._teleport_absolute_request.x = 5.544445
        self._teleport_absolute_request.y = 5.544445
        await self._teleport_absolute_client.call_async(self._teleport_absolute_request)
        
        # Turn on set pen service
        self._set_pen_request.r = int(0)
        self._set_pen_request.g = int(255)
        self._set_pen_request.b = int(0)
        self._set_pen_request.width = int(5)
        self._set_pen_request.off = False
        await self._set_pen_client.call_async(self._set_pen_request)
        
        # Make sure the state is STOPPED
        self._state = STOPPED
        
        # Wrap the response
        response.distance = float(self._waypoints_distance)
            
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