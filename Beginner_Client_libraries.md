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

