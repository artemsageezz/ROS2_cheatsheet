# Beginner: CLI Tools
## Turtlesim, ros2, rqt

* `ros2 pkg executables turtlesim` - check that the package is installed
* `ros2 run turtlesim turtlesim_node` - start turtlesim
* `ros2 run turtlesim turtle_teleop_key` - control turtlesim
* `rqt` - run rqt
* `ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel` - change behavior by remapping (way to control turtle2)