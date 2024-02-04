# Beginner: CLI Tools
## Turtlesim, ros2, rqt [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)

> Using turtlesim and rqt is a great way to learn the core concepts of ROS 2.
* `ros2 pkg executables turtlesim` - check that the package is installed
* `ros2 run turtlesim turtlesim_node` - start turtlesim
* `ros2 run turtlesim turtle_teleop_key` - control turtlesim
* `rqt` - run rqt
* `ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel` - change behavior by remapping (way to control turtle2)

## Understanding nodes [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

> A node is a fundamental ROS 2 element that serves a single, modular purpose in a robotics system.\
>In this tutorial, you utilized nodes created in the `turtlesim` package by running the executables `turtlesim_node` and `turtle_teleop_key`.
>You learned how to use `ros2 node list` to discover active node names and `ros2 node info` to introspect a single node. These tools are vital to understanding the flow of data in a complex, real-world robot system.

* `ros2 list info` - list with names of active nodes
* `ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle` - reassign the name of our `/turtlesim` node. 
* `ros2 node info <node_name>` - more information about nodes.

## Understanding topics [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)


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

## Understanding Services [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

<div align="center">
  <img src="https://docs.ros.org/en/humble/_images/Service-MultipleServiceClient.gif" width="600" height="300"/>
</div>

>Nodes can communicate using services in ROS 2. Unlike a topic - a one way communication pattern where a node publishes information that can be consumed by one or more subscribers - a service is a request/response pattern where a client makes a request to a node providing the service and the service processes the request and generates a response.\
>You generally don’t want to use a service for continuous calls; topics or even actions would be better suited.

* `ros2 service list` - will return a list of all the services currently active in the system. Option `-t` - to see the types of all active services.
* `ros2 service type <service_name>` -  find out the type of a service.
* `ros2 service find <type_name>`- to find all the services of a specific type
* `ros2 interface show <type_name>` - to know the structure of the input arguments.
* `ros2 service call <service_name> <service_type> <arguments>` - call a service (`<arguments>` - YAML format)

## Understanding parameters [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)

> Nodes have parameters to define their default configuration values. You can `get` and `set` parameter values from the command line. You can also save the parameter settings to a file to reload them in a future session.

* `ros2 param list` - will return the nodenamespaces, followed by each node's parameter.
* `ros2 param get <node_name> <parameter_name>` - display the type and current value of a parameter.
* `ros2 param set <node_name> <parameter_name> <value>` - change a parameter’s value at runtime.
* `ros2 param dump <node_name>` -  view all of a node’s current parameter values.
* `ros2 param load <node_name> <parameter_file>` -  load parameters from a file to a currently running node.
* `ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>` - start the same node using your saved parameter values.

## Understanding actions [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

>Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.\
Actions are built on topics and services. Their functionality is similar to services, except actions can be canceled. They also provide steady feedback, as opposed to services which return a single response.\
Actions use a client-server model, similar to the publisher-subscriber model (described in the topics tutorial). An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.

<div align="center">
  <img src="https://docs.ros.org/en/humble/_images/Action-SingleActionClient.gif" width="600" height="300"/>
</div>

* `ros2 node info <node_name>` -  return a list of node’s subscribers, publishers, services, action servers and action clients.
* `ros2 action list` - to identify all the actions in the ROS graph. `-t` - print the type.
* `ros2 action info <action_name>` - introspect the action
* `ros2 interface show <action_type>` - structure of the action type.
* `ros2 action send_goal <action_name> <action_type> <values>` - send an action goal from the command line (yaml format). `--feedback` option let see the feedback.

> A robot system would likely use actions for navigation. An action goal could tell a robot to travel to a position. While the robot navigates to the position, it can send updates along the way (i.e. feedback), and then a final result message once it’s reached its destination.


## Using `rqt_console` to view logs [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)

>`rqt_console` is a GUI tool used to introspect log messages in ROS 2. Typically, log messages show up in your terminal. With `rqt_console`, you can collect those messages over time, view them closely and in a more organized manner, filter them, save them and even reload the saved files to introspect at a different time.\
>Nodes use logs to output messages concerning events and status in a variety of ways. Their content is usually informational, for the sake of the user.

* `ros2 run rqt_console rqt_console` - start `rqt_console`.
* Logger levels:
  *  `Fatal` - messages indicate the system is going to terminate to try to protect itself from detriment.

  *  `Error` messages indicate significant issues that won’t necessarily damage the system, but are preventing it from functioning properly.

  *  `Warn` messages indicate unexpected activity or non-ideal results that might represent a deeper issue, but don’t harm functionality outright.

  *  `Info` messages indicate event and status updates that serve as a visual verification that the system is running as expected.

  *  `Debug` messages detail the entire step-by-step process of the system execution.
* `ros2 run <package> <node_name> --ros-args --log-level <Log_Level>` - set the default logger level.

## Launching nodes [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html)

> In most of the introductory tutorials, you have been opening new terminals for every new node you run. As you create more complex systems with more and more nodes running simultaneously, opening terminals and reentering configuration details becomes tedious.\
>Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.

* `ros2 launch turtlesim multisim.launch.py` - run the following launch file.