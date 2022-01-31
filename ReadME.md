This module is designed to enable direct communication between ROS2 and 3D Slicer\
Testing using ubuntu 20.04 and ROS2 Foxy

This code should be built with colcon as a ROS2 package (run this in the ros2_ws folder): \
Source the setup: source /opt/ros/foxy/setup.bash \
rosdep install -i --from-path src --rosdistro foxy -y \
colcon build --cmake-args -DSlicer_DIR:PATH=/home/laura/Slicer-SuperBuild-Debug/Slicer-build --packages-select EmptyExtensionTemplate

Then open a different terminal: \
-> navigate to Slicer inner build folder \
source ~/ros2_ws/install/setup.bash \
./Slicer

Note: the module should be in the application settings so that it can load
