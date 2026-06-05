..    include:: <isonum.txt>

Getting Started
===============

Pre-requisites
--------------

Before you can start compiling the SlicerROS2 module, you will need:

* Some knowledge of Linux, CMake and ROS 2.

* Ubuntu Linux with `ROS 2 <https://www.ros.org>`_.

* 3D Slicer built from source, this is required to build any C++
  extension, including SlicerROS2.

Versions
--------

SlicerROS2 v1.x: requires Ubuntu 24.04/ROS 2 Jazzy with 3D Slicer 5.10.  When you clone the Slicer repo, make sure to checkout the v5.10 branch/tag.
See :doc:`/pages/compatibility` for the supported release matrix.

.. code-block:: bash

   git clone https://github.com/slicer/slicer
   cd Slicer
   git checkout v5.10.0

Compiling Slicer
----------------

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

If you are comfortable with Docker, you can also use the provided docker image used for continuous integration to build SlicerROS2 module in a containerized environment. The docker file is based on Ubuntu 24.04 and ROS2 Jazzy.  It comes with Slicer pre-installed. It is available on GitHub as a package:

.. code-block:: bash

   docker pull ghcr.io/rosmed/slicer_ros2_module/ci:latest

The docker image is built using the provided Dockerfile in the repository in the ``docker`` directory. You can also build the image yourself using the provided Dockerfile if you want to make modifications to the image.

Compilation
-----------

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

You will first need to "source" the ROS setup script for ROS (Jazzy
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


.. hint::

   If you need to recompile often, you can use ``mold`` instead of the default ``ld``.
   Configure CMake to pass ``-fuse-ld=mold`` to the compiler driver:

   .. code-block:: bash

      cd ~/ros2_ws
      colcon build --cmake-args -DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=mold -DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=mold -DCMAKE_MODULE_LINKER_FLAGS=-fuse-ld=mold


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

Loading the module
------------------

After building your workspace, you should start Slicer using one of the provided ROS 2 launchers. These automatically configure the necessary environment variables and module paths for ``slicer_ros2_module`` and any extensions installed via``manage-extensions.py``.
For more launcher details and examples, see :doc:`/pages/launching-slicer`.

Using ros2 run (Convenient wrapper)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can launch Slicer directly using:

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash
   ros2 run slicer_ros2_module slicer

Passing Slicer arguments
~~~~~~~~~~~~~~~~~~~~~~~~

Additional Slicer arguments can be passed after the launcher command:

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash
   ros2 run slicer_ros2_module slicer --no-splash

Once Slicer is open, SlicerROS2 can be found in the **ROS** category
of the Modules menu (previously it was listed under the **IGT**
category).

After the module loads, a good first workflow is
:doc:`/pages/robot-visualization`.  If you are using MoveIt, continue
with :doc:`/pages/motion-control`.

Managing Extensions
-------------------

If you need additional Slicer extensions (like SlicerIGT), use the
provided management script. This script handles downloading, compiling
(with correct build flags for ROS 2 compatibility), and registering
extensions.

.. code-block:: bash

   ros2 run slicer_ros2_module manage_extensions.py

All files for the extensions will be downloaded and built in the
``install`` directory for slicer in your ROS 2 workspace (``~/ros2_ws/install/slicer_ros2_module/extensions``). The script will also automatically add the extensions to the Slicer module path so they are available when you launch Slicer using the provided ROS 2 launchers (under ``~/ros2_ws/install/slicer_ros2_module/config``).
