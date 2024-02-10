# Beginner: Client libraries

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