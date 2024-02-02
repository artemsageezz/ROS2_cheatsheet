# Beginner: CLI Tools
## Turtlesim, ros2, rqt

> Using turtlesim and rqt is a great way to learn the core concepts of ROS 2.
* `ros2 pkg executables turtlesim` - check that the package is installed
* `ros2 run turtlesim turtlesim_node` - start turtlesim
* `ros2 run turtlesim turtle_teleop_key` - control turtlesim
* `rqt` - run rqt
* `ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel` - change behavior by remapping (way to control turtle2)

## Understanding nodes

> A node is a fundamental ROS 2 element that serves a single, modular purpose in a robotics system.\
>In this tutorial, you utilized nodes created in the `turtlesim` package by running the executables `turtlesim_node` and `turtle_teleop_key`.
>You learned how to use `ros2 node list` to discover active node names and `ros2 node info` to introspect a single node. These tools are vital to understanding the flow of data in a complex, real-world robot system.

* `ros2 list info` - list with names of active nodes
* `ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle` - reassign the name of our `/turtlesim` node. 
* `ros2 node info /my_turtle` - more information about nodes.