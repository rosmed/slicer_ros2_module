Common Errors
=============

Slicer_DIR Not Found
--------------------

If CMake cannot find Slicer, the build may fail with:

.. code-block:: text

   Could not find a package configuration file provided by "Slicer"

Set ``Slicer_DIR`` to the Slicer build directory:

.. code-block:: bash

   colcon build --cmake-args \
     -DSlicer_DIR:PATH=/path/to/Slicer-SuperBuild/Slicer-build \
     -DCMAKE_BUILD_TYPE=Release

After the first successful configure, future builds usually only need:

.. code-block:: bash

   colcon build

Missing ROS Packages
--------------------

If CMake reports a missing ROS package, source ROS and install the package for
the active ROS distribution:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   sudo apt install ros-$ROS_DISTRO-<package-name>

The main package dependencies are listed in ``package.xml``. Additional system
packages are listed in :doc:`compatibility`.

Wrong Python Environment
------------------------

Slicer and ROS 2 use different Python environments. Build from a terminal where
ROS 2 has been sourced, and let the SlicerROS2 CMake configuration switch
between the ROS Python and Slicer Python as needed.

If CMake cache entries point to the wrong Python executable, reconfigure the
package build directory with ``ccmake`` or ``cmake-gui``:

.. code-block:: bash

   ccmake ~/ros2_ws/build/slicer_ros2_module

SlicerROS2 Module Does Not Appear
---------------------------------

Make sure the workspace setup file has been sourced and Slicer was started with
the wrapper:

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash
   ros2 run slicer_ros2_module slicer

Starting the Slicer executable directly can skip the module paths configured by
the ROS 2 workspace.
See :doc:`/pages/launching-slicer` for the recommended launcher and examples.

MoveIt Controls Are Disabled
----------------------------

The motion-control module expects a robot loaded in SlicerROS2 and an available
MoveIt configuration. Confirm that the robot description, SRDF, planning group,
and MoveIt services/actions are available before planning trajectories.
See :doc:`/pages/robot-visualization` for robot loading and
:doc:`/pages/motion-control` for MoveIt setup.

No Joint States Received
------------------------

If joint sliders remain at zero or current-state commands report missing joint
states, confirm that a ``sensor_msgs/msg/JointState`` publisher is active and
that the topic name matches the configured joint-state topic.
