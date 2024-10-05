# CRAZY TURTLE
Demonstration package for ME495.
This README is intentionally vague.
Figuring out how this package works and filling in the details is part of the
exercise. Replace the blanks marked with `${ITEM}` with your answer.
Keep the backticks but remove the `${}`, so `${ITEM}` becomes `My Answer`.
Unless otherwise specified, list the command and all arguments that you passed to it.

## Repository Configuration
1. The `crazy_turtle` git repository consists of the ROS 2 packages `crazy_turtle` and `crazy_turtle_interfaces`.
2. The package `crazy_turtle` is a `ament_python` package.
2. The package `crazy_turtle_interfaces` is a `ament_cmake` package.


## Setup Instructions
1. Build the workspace using `colcon build --symlink-install` so that it is unnecessary to rebuild when python files change.
2. Initialize the ROS environment (i.e., set the necessary ROS environment variables) by executing `source install/setup.bash`
3. Make sure no other ROS nodes are running prior to starting by inspecting the results of `ros2 node list`.
3. Run the launchfile `go_crazy_turtle.launch.xml` by executing `ros2 launch crazy_turtle go_crazy_turtle.launch.xml`
4. When running you can see a visual depiction of the ROS graph using the `rqt_graph` (or `ros2 run rqt_graph rqt_graph`) command.
   The ROS graph, including all topics and node labels, looks like:
   ![The ROS Graph](rosgraph.svg)

## Runtime Information
The `launchfile` from above should be running at all times when executing these commands.
If the nodes launched from the `launchfile` are not running, you will get incorrect results.

5. Use the ROS command `ros2 node list` to list all the nodes that are running.
   The output of the command looks like
   ```
   /mover
   /roving_turtle
   ```
6. Use the ROS command `ros2 topic list` to list the topics
   The output of the command looks like
   ```
   /parameter_events
   /rosout
   /turtle1/cmd_vel
   /turtle1/color_sensor
   /turtle1/pose
   ```

7. Use the ROS command `ros2 topic hz /turtle1/cmd_vel` to verify that the frequency of
   the `/turtle1/cmd_vel` topic is `119.990 Hz`

8. Use the ROS command `ros2 service list` to list the services.
   The output of the command looks like
   ```
   /clear
   /kill
   /mover/describe_parameters
   /mover/get_parameter_types
   /mover/get_parameters
   /mover/get_type_description
   /mover/list_parameters
   /mover/set_parameters
   /mover/set_parameters_atomically
   /reset
   /roving_turtle/describe_parameters
   /roving_turtle/get_parameter_types
   /roving_turtle/get_parameters
   /roving_turtle/get_type_description
   /roving_turtle/list_parameters
   /roving_turtle/set_parameters
   /roving_turtle/set_parameters_atomically
   /spawn
   /switch
   /turtle1/set_pen
   /turtle1/teleport_absolute
   /turtle1/teleport_relative
   ```

9. Use the ROS command `ros2 service type /switch` to determine the type of the `/switch` service, which is `crazy_turtle_interfaces/srv/Switch`.

10. Use the ROS command `ros2 param list` to list the parameters of all running nodes 
    ```
    /mover:
       start_type_description_service
       use_sim_time
       velocity
    /roving_turtle:
       background_b
       background_g
       background_r
       holonomic
       qos_overrides./parameter_events.publisher.depth
       qos_overrides./parameter_events.publisher.durability
       qos_overrides./parameter_events.publisher.history
       qos_overrides./parameter_events.publisher.reliability
       start_type_description_service
       use_sim_time
    ```

11. Use the ROS command `ros2 param describe /mover velocity` to get information about the `/mover` `velocity` parameter, including its type, description, and constraints
    ```
    Parameter name: velocity
       Type: double
       Description: The velocity of the turtle
       Constraints:
    ```

12. Use the ROS command `ros2 interface show crazy_turtle_interfaces/srv/Switch` to retrieve a template/prototype for entering parameters for the `/switch` service on the command line.
    ```
    turtlesim/Pose mixer # use a strange formula to set the new location of the turtle
       float32 x
       float32 y
       float32 theta
       float32 linear_velocity
       float32 angular_velocity
    ---
    float64 x # the new x position of the new turtle
    float64 y # the new y position of the new
    ```

## Package Exploration
1. Use the ROS command `ros2 interface package crazy_turtle_interfaces` to list the interface types defined by `crazy_turtle_interfaces`
   The output of the command looks like
   ```
      crazy_turtle_interfaces/srv/Switch
   ```
2. Use the ROS command `ros2 pkg executables crazy_turtle` to list the executables included with the `crazy_turtle` package
   The output of the command looks like
   ```
   crazy_turtle mover
   ```

## Live Interaction
1. Use the command `ros2 param get /mover velocity` to retrieve the value of the `/mover velocity` parameter, which is `4.5`.
2. The ROS command to call the `/switch` service, and it's output is listed below:
    ```
    ros2 service call /switch crazy_turtle_interfaces/srv/Switch "{mixer: {x: 1.0, y: 2.0, theta: 0.0, angular_velocity: 3.0, linear_velocity: 4.0}}"

    response:
    crazy_turtle_interfaces.srv.Switch_Response(x=5.0, y=4.0)
    ```
3. The `switch` service performs the following actions (in sequence):
    1. It `kills` the current turtle
    2. It then respawns a new turtle at `[x=request.y+angular_velocity, y=request.x*linear_velocity, theta=uniform(-pi, pi)]`, here is `[x=5.0, y=4.0, theta=uniform(-pi, pi)]`
4. What happens to the turtle's motion if you use `ros2 param set /mover velocity 10.0` to change `/mover velocity` to 10? `same`
5. Use the Linux command `pkill mover` to kill the `/mover` node.
6. Use the ROS command `ros2 run crazy_turtle mover --ros-args -p velocity:=10.0 --remap /cmd_vel:=turtle1/cmd_vel` to start the `/mover` node with a velocity of 10. 
    - Be sure to remap `cmd_vel` to `/turtle1/cmd_vel`.
7. What happened to the turtle's velocity after relaunching `mover`? `faster`
