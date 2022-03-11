# C++ based module using `rclcpp`

See README.md

# Python based module using `rclpy`

rclpy is not pure Python, it has binaries that need to match Python
version so that doesn't work as-is since Slicer builds its own Python
(3.9 in latest nightly builds) while rclpy is based on the Ubuntu
system Python (3.8.10).  So we need to recompile some stuff...

## Try to compile Slicer and the super build Python but with a rclpy compatible version

We tried, it gets hard to install all the pip packages Slicer needs
because the hash are set for all the pip installed packages.  We would
need to find all the matching versions and packages.

## Try to compile Slicer using the system Python compatible with rclpy

Tell Slicer the system version is 3.8 or whatever the system version we want to use, edit CMakeLists.txt for Slicer:
`set(Slicer_REQUIRED_PYTHON_VERSION "3.8.10")`

Add packages Slicer wants for the system Python (i.e. match all the pip installed packages in Python super build).  First with the packages available from Ubuntu:
```sh
sudo apt install python3-pip python3-venv python3-scipy python3-dicom python3-git python3-pycparser python3-cffi
```
Then add a few more using pip:
```sh
sudo pip3 install dicomweb_client couchdb
```

Build with the system Python, in build directory:
```sh
cmake ../Slicer -DSlicer_USE_SYSTEM_python=ON
```

Then use `sudo` to build because the build needs to install some new Python modules...
```sh
sudo make -j8
```

And we're running into some pythread errors!
```c++
#   error "Require native threads. See https://bugs.python.org/issue31370"
```

## New approach: can we use the Slicer Python to compile rclpy?

```sh
mkdir -p ~/rclpy_ws/src
cd ~/rcy_ws/src/
git clone https://github.com/ros2/rclpy
source /opt/ros/foxy/setup.bash 
cd ..
colcon build
```

That failed even with system python.  So not sure what is going on.

To change which Python to use:
```sh
export PYTHONPATH=/home/anton/devel/Slicer-SuperBuild-Debug/python-install/lib/python3.9/site-packages
export LD_LIBRARY_PATH=/home/anton/devel/Slicer-SuperBuild-Debug/python-install/lib:$LD_LIBRARY_PATH
export PATH=/home/anton/devel/Slicer-SuperBuild-Debug/python-install/bin:$PATH
```

Then we can use the Slicer python, including `pip`.  At that point,
one needs to add `ament_package` to the `src` directory (`git clone
https://github.com/ament/ament_package`) since it can't be installed
over pip.  I got stuck on Foxy with
`rcl_logging_interfaceConfig.cmake" missing.  So I removed Foxy and
tried Galactic.  On Galactic, we also need `catkin_pkg` which can be
installed with `pip install catkin_pkg`.  Now stuck on compilation
errors for `rclpy` re. cast of const/non-const ptr.  Note that in
`src/rclpy` I tried to checkout `galactic` when working on Galactic,
foxy ...
