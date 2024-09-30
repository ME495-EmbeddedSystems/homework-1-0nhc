import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Empty

MOVING = 0
STOPPED = 1

class WaypointNode(Node):

    def __init__(self):
        super().__init__('waypoint_node')
        # Set logger level to DEBUG
        # self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Declare timer frequency parameter, default to 90 Hz
        self.declare_parameter('frequency', 90.0)
        self._timer_frequency = self.get_parameter("frequency").get_parameter_value().double_value
        
        # Enable dynamic reconfiguration for parameters
        self.add_on_set_parameters_callback(self._parameter_callback)
        
        # Setup ROS 2 timer
        self._timer = self.create_timer(1.0/self._timer_frequency, self._timer_callback)
        
        # Setup ROS 2 Service
        self._servive = self.create_service(Empty, 'toggle', self._toggle_callback)
        
        # Initialize state
        self._initial_state = STOPPED
        self._state = self._initial_state
        

    def _timer_callback(self):
        if(self._state == MOVING):
            self.get_logger().debug('Issuing Command!')
            # While moving the turtle should leave a visual trail behind to show where it has been
    
    
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