v1.0
====

* Last version known to work with Ubuntu 20.04
* Tested on 20.04, 22.04 and 24.04
* Binaries are now installed in Slicer build tree so users don't have
  to modify the module search path in Slicer
* Added code generation to automatically create vtk object mimicking
  ROS messages, also generates conversion methods between ROS and
  Slicer. New subscribers and publishers can be added in the
  `MRML/CMakeLists.txt`
* For robot visualization, added "fixed frame" to allow setting a
  different name for the reference frame
* crtl+c now stops slicer to be more ROS-like
* Tests now detect the ROS version and should work on Galactic, Humble
  and Jazzy
* Added methods to list registered (existing) publisher and subscriber nodes
* `CreateAndAdd{Publisher,Subscriber}Node` overloaded to accept short
  name (e.g. `String` instead of `vtkMRMLROS2PublisherStringNode`


v0.9
====

SlicerROS2 now supports ROS publishers and subscribers, Tf2 broadcasts
and lookups as well as parameters (client). The robot visualization is
built on top of the parameter client (to retrieve the URDF) as well as
Tf2 lookups (to move the robot's links).

Each ROS communication mechanism is encapsulated in a MRML node. These
can be used in C++ as well as Python. Some simple unit tests are
included in the accompanying module ROS2Tests.

Detailed documentation is available on readthedocs:
https://slicer-ros2.readthedocs.io/en/latest/
