To build - do these commands:
Source the setup: source /opt/ros/foxy/setup.bash
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --cmake-args -DSlicer_DIR:PATH=/home/laura/Slicer-SuperBuild-Debug/Slicer-build --packages-select EmptyExtensionTemplate

ONLY DO THAT IN THE ros2_ws folder!! ^^^

You need to source the setup in the terminal where you run Slicer as well to avoid dynamic library linking error:
source ~/ros2_ws/install/setup.bash
(you might get a warning that you can ignore)
then do: ./Slicer

- add module to the application settings so Slicer shows in under Examples header
