# Beginner: Client libraries

[Using `colcon` to build packages](#using-colcon-to-build-packages-docs)\
[Creating a workspace](#creating-a-workspace-docs)\
[Creating a package](#creating-a-package-docs)\
[Writing a simple publisher and subscriber (Python)](#writing-a-simple-publisher-and-subscriber-python-docs)

## Using `colcon` to build packages [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

>`colcon` is an iteration on the ROS build tools `catkin_make`, `catkin_make_isolated`, `catkin_tools` and `ament_tools`

* By default it will create the following directories as peers of the `src` directory:
  * The `build` directory will be where intermediate files are stored. For each package a subfolder will be created in which e.g. CMake is being invoked.

  * The `install` directory is where each package will be installed to. By default each package will be installed into a separate subdirectory.

  * The `log` directory contains various logging information about each colcon invocation.

* The setup script provided by a binary installation or a source installation ie. another colcon workspace - **underlay**. Our workspace, `ros2_ws`, will be an **overlay** on top of the existing ROS 2 installation.

* `colcon build --symlink-install` - build the workspace. Run in the root of the workspace.

* `source install/setup.bash` - source the environment. When colcon has completed building successfully, the output will be in the `install` directory.

* If you do not want to build a specific package place an empty file named `COLCON_IGNORE` in the directory and it will not be indexed.

## Creating a workspace [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

> A workspace is a directory containing ROS 2 packages. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace in the terminal you plan to work in. This makes ROS 2’s packages available for you to use in that terminal.\
>The overlay gets prepended to the path, and takes precedence over the underlay.\
>Using overlays is recommended for working on a small number of packages, so you don’t have to put everything in the same workspace and rebuild a huge workspace on every iteration.

* Best practice is to create a new directory for every new workspace. Example `ros2_ws`.

* `git clone https://github.com/ros/ros_tutorials.git -b humble` - run in the `ros2_ws/src` directory. `ros_tutorials` is cloned.

* `rosdep install -i --from-path src --rosdistro humble -y` - from the root of workspace (`ros2_ws`). Before building the workspace, to resolve the package dependencies.

* `colcon build` - from the root of workspace (`ros2_ws`)  build packages.\
Other useful arguments for `colcon build`:

  * `--packages-up-to` builds the package you want, plus all its dependencies, but not the whole workspace (saves time)

  * `--symlink-install` saves you from having to rebuild every time you tweak python scripts

  *  `--event-handlers console_direct+` shows console output while building (can otherwise be found in the `log` directory)

Before sourcing the overlay, it is very important **open a new terminal**, separate from the one where built the workspace
* `source /opt/ros/humble/setup.bash` - source your main ROS 2 environment as the “underlay”.

* `cd ~/ros2_ws` - go into the root of your workspace.

* `source install/local_setup.bash` - in the root, source your overlay.

You can modify `turtlesim` in your overlay by editing the title bar on the turtlesim window. To do this, locate the `turtle_frame.cpp` file in `~/ros2_ws/src/ros_tutorials/turtlesim/src`. Open `turtle_frame.cpp` with your preferred text editor.\
On line 52 you will see the function `setWindowTitle("TurtleSim");`. Change the value `"TurtleSim"` to `"MyTurtleSim"`, and save the file.

...

You can see that modifications in the overlay did not actually affect anything in the underlay.

## Creating a package [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

> A package is an organizational unit for your ROS 2 code. If you want to be able to install your code or share it with others, then you’ll need it organized in a package. With packages, you can release your ROS 2 work and allow others to build and use it easily.\
>Package creation in ROS 2 uses ament as its build system and colcon as its build tool. You can create a package using either CMake or Python, which are officially supported, though other build types do exist.

What makes up a ROS 2 package?
* `package.xml` file containing meta information about the package

* `resource/<package_name>` marker file for the package

* `setup.cfg` is required when a package has executables, so `ros2 run` can find them

* `setup.py` containing instructions for how to install the package

* `<package_name>` - a directory with the same name as your package, used by ROS 2 tools to find your package, contains `__init__.py `

```
my_package/
      package.xml
      resource/my_package
      setup.cfg
      setup.py
      my_package/
```

A single workspace can contain as many packages as you want, each in their own folder. Best practice is to have a `src` folder within your workspace, and to create your packages in there.

```
workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/
```

* `ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>` (python) - creating a new package.\
exampe: `ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node my_package`

* `colcon build` -  build your packages. To build only the `my_package` package `colcon build --packages-select my_package`.

* `ros2 run my_package my_node` - to run the executable you created using the `--node-name` argument during package creation.

Inside `ros2_ws/src/my_package`, you will see the files and folders that ros2 pkg create automatically generated:
`my_package  package.xml  resource  setup.cfg  setup.py  test`\
`my_node.py` is inside the my_package directory.

In package.xml fields **description** and **license** declaration are not automatically set, but are required if you ever want to release your package. The **maintainer** field may also need to be filled in.

Below the license tag, you will see some tag names ending with `_depend`. This is where your `package.xml` would list its dependencies on other packages, for colcon to search for.

The `setup.py` file contains the same **description**, **maintainer** and **license** fields as `package.xml`, so you need to set those as well. They need to match exactly in both files. The version and name (`package_name`) also need to match exactly, and should be automatically populated in both files.

## Writing a simple publisher and subscriber (Python) [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

>The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

* `ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub` - package creation command (in `ros2_ws/src` directory).

* `wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py` - Download the example talker code (in `ros2_ws/src/py_pubsub/py_pubsub`).

In `package.xml` fill in the `<description>`, `<maintainer>` and `<license>` tags and add the following dependencies corresponding to your node’s import statements:

```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

In `setup.py` match the `maintainer`, `maintainer_email`, `description` and `license` fields to your `package.xml` and add the following line within the `console_scripts` brackets of the `entry_points` field:

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```
The contents of the `setup.cfg` file should be correctly populated automatically, like so:

```
[develop]
script_dir=$base/lib/py_pubsub
[install]
install_scripts=$base/lib/py_pubsub
```

* `wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py` - download subscriber node (in `ros2_ws/src/py_pubsub/py_pubsub` directory)

In `setup.py` add the entry point for the subscriber node below the publisher’s entry point. The entry_points field should now look like this:

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```

* `rosdep install -i --from-path src --rosdistro humble -y` - in the root of your workspace (`ros2_ws`)to check for missing dependencies before building.

* `colcon build --packages-select py_pubsub` - in the root of your workspace, `ros2_ws`, build your new package.

* `source install/setup.bash` - in **NEW** terminal navigate to `ros2_ws` source the setup files.

* `ros2 run py_pubsub talker` - run the talker node.

* `ros2 run py_pubsub listener` - run the listener node.

