v1.2
====

* Added trajectory path visualization with display toggles and configurable path colors
* Added obstacle transform tracking and TF2 broadcasting for live MoveIt planning scene updates
* Added MoveIt attached tool support with planning scene tool publishing and a custom tool example
* Added Kinova Gen3 Lite launch files for robot visualization, simulation, and control
* Improved service clients with response events, wait helpers, and non-blocking workflow documentation
* Improved MoveIt IK helpers with clearer return values and seed joint validation
* Added extension manager support for updating and recompiling installed extensions
* Added compatibility, common errors, launching Slicer, and expanded example documentation
* Fixed CI setup to free disk space before pulling the large Docker image
* Added GitHub Actions CI using a reusable Docker image with ROS2 Jazzy and Slicer 5.10
* Added Docker image build and push instructions for maintaining the CI image
* Added command-line wrappers to launch Slicer and run the SlicerROS2 test suite from ROS2
* Added support for running Python scripts through the Slicer ROS2 launcher
* Added documentation examples for point cloud publishing/subscribing and MoveIt obstacle publishing
* Improved ROS2MotionControl trajectory generation, end-effector selection, and UI parameters
* Added and documented QoS tests for compatible and incompatible publisher/subscriber settings
* Updated release metadata, package dependencies, and documentation links

v1.1
====

* Tested on Ubuntu 24.04 with ROS2 Jazzy
* Added ROS2MotionControl module with MoveIt integration for trajectory planning and execution
* Added interactive 3D control and joint control interfaces for robot manipulation
* Added extension manager script with GUI to download, build, and install extensions from git repos
* Added launch file to start Slicer with correct paths
* Build is now in colcon workspace, no files dumped in the Slicer build tree
* Added support for `PointCloud` and `PointCloud2` message types in subscribers
* Robot cleanup improved: added `RemoveFromROS2Node` method and fixes to avoid crashes on removal
* Added DAE mesh loading, now creates multiple meshes per link when multiple colors are used (no need to manually update URDF files)
* Module is now listed under the **ROS** category in Slicer's Modules menu

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
