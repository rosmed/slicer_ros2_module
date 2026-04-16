..    include:: <isonum.txt>

"""""""""""""""
Getting Started
"""""""""""""""
==============
Pre-requisites
==============

Before you can start compiling the SlicerROS2 module, you will need:

* Some knowledge of Linux, CMake and ROS 2.

* Ubuntu Linux with `ROS 2 <https://www.ros.org>`_.

* Slicer 3D built from source, this is required to build any C++
  extension, including SlicerROS2.

========
Versions
========

SlicerROS2 v1.x: requires Ubuntu 24.04/ROS 2 Jazzy with Slicer 3D 5.10

================
Compiling Slicer
================

See `Slicer build instructions
<https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/linux.html>`_
for generic instructions.

For the SlicerROS2 module, there are few things to keep in mind before
you start compiling Slicer:

* Qt installed using Ubuntu. The build instructions for Slicer
  sometimes recommend installing Qt from the Qt site, but that leads
  to major issues when compiling against ROS2.  So don't re-install
  Qt from the Qt site.  Use ``apt install``!

* Make sure you use the system/native OpenSSL and bzip2 libraries
  otherwise you'll get some errors when compiling the Slicer ROS2
  module (as opposed to the Slicer super build ones).  You will need
  to do the following after you ran CMake for the first time.  In the
  Slicer build directory, set ``Slicer_USE_SYSTEM_OpenSSL`` and
  ``Slicer_USE_SYSTEM_bzip2`` to ``ON`` using ``cmake
  . -DSlicer_USE_SYSTEM_OpenSSL=ON -DSlicer_USE_SYSTEM_bzip2=ON
  -DCMAKE_BUILD_TYPE=Release`` or ``ccmake``.

* Compiling Slicer from source takes time, plan a few hours
  ahead. Also, avoid using ``make -j`` without any limit. The
  compilation process requires a fair amount of memory and is likely
  to crash your computer. Using about half the number of cores
  available seems to help. For example, use ``make -j4`` for a Intel
  i9 processor.

* To get a specific version of Slicer from GitHub, first clone: ``git
  clone https://github.com/slicer/slicer`` and then checkout the
  version using ``git checkout v5.10.0``.


===========
Compilation
===========

This code should be built with ``colcon`` as a ROS package.
``colcon`` is usually installed along ROS but if it isn't, install
it with ``sudo apt install python3-colcon-common-extensions``.  For
now, we will assume the ROS workspace directory is ``~/ros2_ws`` and
the source code for this module has been cloned under
``~/ros2_ws/src/slicer_ros2_module``.

Other ROS packages that might not be installed by default. You can
install all remaining core components and libraries by doing:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   sudo apt install ros-$ROS_DISTRO-rclcpp ros-$ROS_DISTRO-tf2 ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-kdl-parser ros-$ROS_DISTRO-urdf ros-$ROS_DISTRO-std-msgs ros-$ROS_DISTRO-std-srvs ros-$ROS_DISTRO-geometry-msgs ros-$ROS_DISTRO-sensor-msgs ros-$ROS_DISTRO-trajectory-msgs ros-$ROS_DISTRO-object-recognition-msgs ros-$ROS_DISTRO-rosbag2-interfaces ros-$ROS_DISTRO-turtlesim ros-$ROS_DISTRO-moveit-msgs ros-$ROS_DISTRO-moveit-core ros-$ROS_DISTRO-moveit-ros-planning ros-$ROS_DISTRO-moveit-ros-planning-interface liborocos-kdl-dev libassimp-dev mold

* You will first need to "source" the ROS setup script for ROS (Jazzy
in this example):

.. code-block:: bash

    source /opt/ros/jazzy/setup.bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/rosmed/slicer_ros2_module

Then build the module using ``colcon`` while providing the path to your
Slicer build directory ``Slicer_DIR``:

.. code-block:: bash

    cd ~/ros2_ws
    colcon build --cmake-args -DSlicer_DIR:PATH=/home/your_user_name_here/something_something/Slicer-SuperBuild-Debug/Slicer-build -DCMAKE_BUILD_TYPE=Release


The option ``--cmake-args -DSlicer_DIR...`` is only needed for the
first ``colcon`` call.  For future builds, you can just use ``colcon build``.

If the ``Slicer_DIR`` is not set properly (or you simply forgot), you
should see the following error messages"

.. code-block:: bash

  Could not find a package configuration file provided by "Slicer" with any
  of the following names:

  SlicerConfig.cmake
  slicer-config.cmake

At that point, you don't need to clean your ROS workspace.  You can
fix the issue by running CMake on the build directory for the Slicer
module ``ccmake ~/ros2_ws/build/slicer_ros2_module``.  In CMake, set
``Slicer_DIR`` to point to your Slicer build directory then hit ``c``
to configure until you can hit ``g`` to generate the makefiles.  If
you prefer a graphical interface, you can use ``cmake-gui`` instead of
``ccmake``.  Once ``Slicer_DIR`` is set, try ``colcon build`` again
(after ``cd ~/ros2_ws``).

==================
Loading the module
==================

After sourcing your workspace, use the provided launcher to start
Slicer with the correct module paths automatically configured:

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash # or whatever your ROS workspace is
   ros2 launch slicer_ros2_module slicer.launch.py

During the ``colcon`` build, the files required for the SlicerROS2 module are
installed in the Slicer build directory so the user doesn't have to
change the module paths to load the newly created module.
