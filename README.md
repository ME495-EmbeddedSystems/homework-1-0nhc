# ME495 Embedded Systems Homework 1
Author: Zhengxiao Han
1. Use `ros2 launch turtle_control waypoints.launch.xml` to run the code
2. The `ros2 service call /load turtle_interfaces/srv/Waypoints "{waypoints: [{x: 1.4, y: 1.6}, {x: 2.2, y: 9.4}, {x: 7.2, y: 6.1}, {x: 4.0, y: 2.6}, {x: 8.2, y: 1.5}, {x: 4.1, y: 5.3}]}"` service loads waypoints for the turtle to follow
3. The `ros2 service call /toggle std_srvs/srv/Empty "{}"` starts and stops the turtle.
4. Here is a video of the turtle in action.
   https://github.com/user-attachments/assets/ab9ca59a-80af-4396-b4ff-fb0d8419d36e
