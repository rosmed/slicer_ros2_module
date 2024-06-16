..    include:: <isonum.txt>

"""""""""""""""
Getting Started
"""""""""""""""
==============
Pre-requisites
==============

Before you can start compiling the SlicerROS2 module, you will need:

* Some knowledge of Linux, CMake and ROS 2.
  
* Ubuntu Linux with `ROS 2 <https://www.ros.org>`_.  This module has
  been developed and tested using Ubuntu 20.04 with ROS Galactic.  It
  might also work on Ubuntu 22.04 with ROS Humble.
  
* Slicer 3D built from source is required to build an extension.

  * Ubuntu 20.04 only.  Instructions from the Slicer site recommend to
    install a newer version of Qt on Ubuntu 20.04.  This makes the
    builds with ROS2 quite difficult so instead, we recommend to use
    this patch: `SlicerROS2-Patch <https://github.com/LauraConnolly/Slicer/tree/SlicerROS2_patch>`_  and use the Qt libraries installed by
    default along Ubuntu (this was tested with Qt 5.12.8).  After you cloned the Slicer sources, make
    sure you checkout the branch SlicerROS2_patch using ``git checkout SlicerROS2_patch``.
    
  * Before you start compiling Slicer, make sure we use the
    system/native OpenSSL libraries otherwise you'll get some errors
    when compiling the Slicer ROS 2 module.  You will need to do the
    following after you ran CMake for the first time.  In the Slicer
    build directory, set ``Slicer_USE_SYSTEM_OpenSLL`` to ``ON`` using
    ``cmake . -DSlicer_USE_SYSTEM_OpenSSL=ON -DCMAKE_BUILD_TYPE=Release`` or ``ccmake``.

  See `Slicer build instructions <https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/linux.html>`_.

* Remember the build directory for Slicer, it will be needed to
  compile the Slicer ROS 2 module.

.. note:: If you need to build Slicer from old sources, make sure
  ``CMAKE_CXX_STANDARD`` is set to ``14`` (required to compile Slicer
  code along ROS 2).

===========
Compilation
===========

This code should be built with ``colcon`` as a ROS 2 package.
``colcon`` is usually installed along ROS 2 but if it isn't, install
it with ``sudo apt install python3-colcon-common-extensions``.  For
now, we will assume the ROS workspace directory is ``~/ros2_ws`` and
the source code for this module has been cloned under
``~/ros2_ws/src/slicer_ros2_module``.

You will first need to "source" the ROS setup script for ROS 2 (Galactic
in this example):

.. code-block:: bash

    source /opt/ros/galactic/setup.bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/rosmed/slicer_ros2_module

Then build the module using `colcon` while providing the path to your
Slicer build directory ``Slicer_DIR``:

.. code-block:: bash

    cd ~/ros2_ws
    colcon build --cmake-args -DSlicer_DIR:PATH=/home/your_user_name_here/something_something/Slicer-SuperBuild-Debug/Slicer-build -DCMAKE_BUILD_TYPE=Release


The option ``--cmake-args -DSlicer_DIR...`` is only needed for the
first ``colcon`` call.  For future builds, you can revert back to just
using ``colcon build``.

Note that the build directory name is ``ROS2``, not
``slicer_ros2_module``.  You will see some CMake warnings mentioning
that the module's name (``ROS2``) doesn't follow the ROS2 package
naming convention.  ROS2 expects all packages to use the "snake_case"
naming convention but we decided to ignore this to follow the
VTK/Slicer naming convention, i.e. "CamelCase".

If the ``Slicer_DIR`` is not set properly (or you simply forgot), you
should see the following error messages"

.. code-block:: bash

  Could not find a package configuration file provided by "Slicer" with any
  of the following names:

  SlicerConfig.cmake
  slicer-config.cmake

At that point, you don't need to clean your ROS workspace.  You can
fix the issue by running CMake on the build directory for the Slicer
module (``ROS2``) ``ccmake ~/ros2_ws/build/ROS2``.  In CMake, set
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

The first time you run Slicer, you need to add the module directory in
the application settings so that the module can be dynamically loaded.

To do so, open Slicer and navigate through the menus: `Edit` |rarr|
`Application Settings` |rarr| `Modules` |rarr| `Additional module
paths` |rarr| `Add`.  The path to add is based on your ROS workspace
location as well as the Slicer version (5.3 in this example).  It should look like:

.. code-block:: bash

    ~ros2_ws/build/ROS2/lib/Slicer-5.3/qt-loadable-modules

At that point, Slicer will offer to restart.  Do so and then load the
module using the drop down menu: `Modules` |rarr| `IGT` |rarr| *ROS2*
