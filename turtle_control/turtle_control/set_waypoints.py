import rclpy
from rclpy.node import Node
from turtle_interfaces.srv import Waypoints
from geometry_msgs.msg import Point

MOVING = 0
STOPPED = 1

class SetWaypointsNode(Node):

    def __init__(self):
        super().__init__('set_waypoints_node')
        
        # Setup ROS 2 Client
        # Reset turtlesim client
        self._set_waypoints_client = self.create_client(Waypoints, 'load')
        while not self._set_waypoints_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for waypoint_node's service: /load")
        self._set_waypoints_request = Waypoints.Request()
        
        # Initialize waypoints
        self._waypoints = [[1.4, 1.6], [2.2, 9.4], [7.2, 6.1], [4.0, 2.6], [8.2, 1.5], [4.1, 5.3]]
        
        # Set waypoints
        for idx in range(len(self._waypoints)):
            point = self._waypoints[idx]
            self.get_logger().info(str(point))
            point_msg = Point()
            point_msg.x = point[0]
            point_msg.y = point[1]
            point_msg.z = 0.0
            self._set_waypoints_request.waypoints.append(point_msg)
        self._set_waypoints_client.call(self._set_waypoints_request)
        
        self.destroy_node()
        

def main(args=None):
    rclpy.init(args=args)
    SetWaypointsNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()