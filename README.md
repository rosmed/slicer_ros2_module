This module is designed to enable direct communication between ROS2 and 3D Slicer

## Pre-requisites

* ROS 2 (tested using ubuntu 20.04 and ROS2 Foxy), see www.ros.org.
* Slicer built from source (required to build an extension).  Remember the build path for Slicer, it will be needed to compile the module.
* Make sure we use the system OpenSSL libraries otherwise you'll get some errors when compiling the ROS 2 part.  After you ran CMake, in the build tree, set `Slicer_USE_SYSTEM_OpenSLL` `ON` using `cmake . -DSlicer_USE_SYSTEM_OpenSSL=ON`.
* **Older Slicer** Make sure `CMAKE_CXX_STANDARD` is set to `14` (required to compile Slicer code along ROS 2).

## Compilation

This code should be built with colcon as a ROS2 package.  For now, we will assume the ROS workspace directory is `~/ros2_ws` and the source code for this module is under `~/ros2_ws/src/SlicerRos2`.

Source the ROS setup script:
```sh
source /opt/ros/foxy/setup.bash
```

Build the module using `colcon` while providing the path to your Slicer build directory:
```sh
cd ~/ros2_ws
colcon build --cmake-args -DSlicer_DIR:PATH=/home/your_user_name_here/something_something/Slicer-SuperBuild-Debug/Slicer-build
```
Note: the `-DSlicer_DIR...` option is only needed for the first `colcon build`

## Using the module

In a terminal navigate to your Slicer inner build folder (`cd`).  Then:
```sh
source ~/ros2_ws/install/setup.bash # or whatever your ROS 2 workspace is
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/foxy/lib # change for your ROS 2 distribution
./Slicer
```

Note: the module directory should be in the application settings so that it can be loaded.  To do so, open Slicer and navigate through the menus: `Edit` -> `Application Settings` -> `Modules` -> `Additional module paths` ->  `Add`:
```sh
~ros2_ws/build/SlicerRos2/lib/Slicer-4.13/qt-loadable-modules
```