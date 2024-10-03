import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoints
from turtlesim.srv import TeleportAbsolute, SetPen

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
        # Remove '/'
        while(self._robot_name[0] == '/'):
            self._robot_name = self._robot_name[1:]
            self.get_logger().info(self._robot_name)
        
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
        
        # Initialize state
        self._state = STOPPED
        # Initialize waypoints
        self._waypoints = [[1.4, 1.6], [2.2, 9.4], [7.2, 6.1], [4.0, 2.6], [8.2, 1.5], [4.1, 5.3]]
        

    async def _timer_callback(self):
        if(self._state == MOVING):
            self.get_logger().debug('Issuing Command!')
            # While moving the turtle should leave a visual trail behind to show where it has been
    
    
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
        # Read waypoints from request
        for point in request.waypoints:
            # Draw an X
            await self._draw_an_X(point.x, point.y)

        # Send reset turtlesim request
        # self._reset_turtlesim_client.call_async(self._reset_turtlesim_request)
        
        # Go back to zero position
        self._teleport_absolute_request.x = 5.544445
        self._teleport_absolute_request.y = 5.544445
        await self._teleport_absolute_client.call_async(self._teleport_absolute_request)
        
        # Turn on set pen service
        self._set_pen_request.off = False
        await self._set_pen_client.call_async(self._set_pen_request)
            
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