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
* `ros2 node info <node_name>` - more information about nodes.

## Understanding topics

<div align="center">
  <img src="https://docs.ros.org/en/humble/_images/Topic-MultiplePublisherandMultipleSubscriber.gif" width="600" height="300"/>
</div>

>Nodes publish information over topics, which allows any number of other nodes to subscribe to and access that information. In this tutorial you examined the connections between several nodes over topics using rqt_graph and command line tools.

* `rqt_graph` - run rqt_graph
* `ros2 topic list` - return a list of all the topics currently active in the system. Option `-t` return the same list, but with the topic type
* `ros2 topic echo <topic_name>` - to see the data being published on a topic
* `ros2 topic info <topic_name>` - the way to look on the info about topic
* `ros2 interface show <msg type>` - what structure of data the message expects
* `ros2 topic pub <topic_name> <msg_type> '<args>'` -  publish data to a topic directly from the command line (`'<args>'` - YAML format). `--once` -  is an optional argument meaning “publish one message then exit”. `--rate 1` - option which tells to publish the command in a steady stream at 1 Hz.
* `ros2 topic hz <topic_name>` - the rate at which data is published using.