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

* Qt installed using Ubuntu.  The build instructions for Slicer
  sometimes recommend installing Qt from the Qt site, but that leads
  to major issues when compiling against ROS2.  So don't re-install
  Qt from the Qt site.  Use ``apt install``!

* Slicer 3D built from source, this is required to build any C++
  extension, including SlicerROS2.

  See also `Slicer build instructions <https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/linux.html>`_.

  .. warning::

     Before you start compiling Slicer, make sure you use the
     system/native OpenSSL libraries otherwise you'll get some errors
     when compiling the Slicer ROS 2 module (as opposed to the Slicer
     super build ones).  You will need to do the following after you
     ran CMake for the first time.  In the Slicer build directory, set
     ``Slicer_USE_SYSTEM_OpenSLL`` to ``ON`` using ``cmake
     . -DSlicer_USE_SYSTEM_OpenSSL=ON -DCMAKE_BUILD_TYPE=Release`` or
     ``ccmake``.

  .. note::

     Compiling Slicer from source takes time, plan a few hours
     ahead. Also, avoid using ``make -j`` without any limit. The
     compilation process requires a fair amount of memory and is
     likely to crash your computer. Using about half the number of
     cores available seems to help. For example, use ``make -j4`` for
     a Intel i9 processor. Compiling the SlicerROS2 module itself can
     take up to 10 minutes.

* Remember the build directory for Slicer, it will be needed to
  compile the Slicer ROS 2 module.

========
Versions
========

* *Recommended*: SlicerROS2 v1.0: requires Ubuntu 24.04/ROS 2 Jazzy
  with Slicer 5.8 (should also work with Slicer 5.6)

* SlicerROS2 v0.9: requires Ubuntu 20.04/ROS Galactic, Ubuntu
  22.04/ROS2 Humble or Ubuntu 24.04/ROS2 Jazzy with Slicer 5.6

Older versions and compilation tricks:

* If you need to use Ubuntu 22.04/ROS 2 Humble, SlicerROS2 v1.0+ will
  fail to compile unless you comment out the the rosbag2 service
  clients ``Play``, ``Stop`` and ``SplitBagFile`` in the macro call
  ``generate_ros2_nodes``, file ``MRML/CMakeLists.txt``.

* When compiling on Ubuntu 20.04, Slicer 5.2.2 is known to compile
  without any issues.  If you need to use a more recent version of
  Slicer, you might have to edit the Slicer code to replace a few
  ``QLatin1String`` to ``QString``.  You can replace all occurences in
  the Slicer source directory using ``find . -not -path '*/\.git/*'
  \( -name '*.cxx*' -o -name '*.h*' \) -exec sed -i
  's/QLatin1String/QString/g' '{}' \;``


===========
Compilation
===========

This code should be built with ``colcon`` as a ROS 2 package.
``colcon`` is usually installed along ROS 2 but if it isn't, install
it with ``sudo apt install python3-colcon-common-extensions``.  For
now, we will assume the ROS workspace directory is ``~/ros2_ws`` and
the source code for this module has been cloned under
``~/ros2_ws/src/slicer_ros2_module``.

Other ROS packages that might not be installed by default.  Replace
``<distro>`` by the ROS distribution's name (e.g. ``humble``,
``jazzy``...).

* ``ros-<distro>-moveit-msgs``

You will first need to "source" the ROS setup script for ROS 2 (Galactic
in this example):

.. code-block:: bash

    source /opt/ros/galactic/setup.bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/rosmed/slicer_ros2_module

Then build the module using ``colcon`` while providing the path to your
Slicer build directory ``Slicer_DIR``:

.. code-block:: bash

    cd ~/ros2_ws
    colcon build --cmake-args -DSlicer_DIR:PATH=/home/your_user_name_here/something_something/Slicer-SuperBuild-Debug/Slicer-build -DCMAKE_BUILD_TYPE=Release


The option ``--cmake-args -DSlicer_DIR...`` is only needed for the
first ``colcon`` call.  For future builds, you can revert back to just
using ``colcon build``.

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

You will first need to make sure the environment variables are set
properly so the Slicer ROS 2 module can locate all the ROS 2 resources
(dynamic libraries and other ROS 2 packages you might need to access):

.. code-block:: bash

  source ~/ros2_ws/install/setup.bash # or whatever your ROS 2 workspace is

In the same terminal navigate (``cd``) to your Slicer inner build
directory and start Slicer.  If you followed the Slicer build
instructions, this should look like:

.. code-block:: bash

  cd ~/something_something/Slicer-SuperBuild/Slicer-build
  ./Slicer

During the ``colcon`` build, the files required for the SlicerROS2 module are
installed in the Slicer build directory so the user doesn't have to
change the module paths to load the newly created module.
