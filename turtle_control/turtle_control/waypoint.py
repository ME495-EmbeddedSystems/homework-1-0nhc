import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoints

MOVING = 0
STOPPED = 1

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
        # self._reset_turtlesim_client = self.create_client(Empty, 'reset')
        # while not self._reset_turtlesim_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Waiting for turtlesim's service: /reset")
        # self._reset_turtlesim_request = Empty.Request()
        
        # Initialize state
        self._state = STOPPED
        

    async def _timer_callback(self):
        if(self._state == MOVING):
            self.get_logger().debug('Issuing Command!')
            # While moving the turtle should leave a visual trail behind to show where it has been
    
    
    def _load_callback(self, request, response):
        # Send reset turtlesim request
        self._reset_turtlesim_client.call_async(self._reset_turtlesim_request)
            
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