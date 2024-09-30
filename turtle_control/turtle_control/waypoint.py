import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String


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
        self.timer = self.create_timer(1.0/self._timer_frequency, self._timer_callback)
        

    def _timer_callback(self):
        self.get_logger().debug('Debug: Issuing Command!')
    
    
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