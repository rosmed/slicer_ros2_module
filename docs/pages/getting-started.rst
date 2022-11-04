"""""""""""""""
Getting Started
"""""""""""""""
==============
Pre-requisites
==============

* `ROS 2 <https://www.ros.org>`_ (tested using ubuntu 20.04 and ROS2 Foxy or Galactic).
* Slicer built from source is required to build an extension, see `Slicer build instructions <https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/linux.html>`_. **Remember the build directory for Slicer, it will be needed to compile the Slicer ROS 2 module.**

.. warning::

    Make sure we use the system/native OpenSSL libraries 
    otherwise you'll get some errors when compiling the 
    Slicer ROS 2 module.  After you ran CMake, in the Slicer 
    build directory, set ``Slicer_USE_SYSTEM_OpenSLL`` ``ON`` 
    using ``cmake . -DSlicer_USE_SYSTEM_OpenSSL=ON`` or ``ccmake``.

.. note::

  *Older Slicer* Make sure ``CMAKE_CXX_STANDARD`` is set to ``14`` (required to compile Slicer code along ROS 2).

===========
Compilation
===========

This code should be built with ``colcon`` as a ROS2 package.  
For now, we will assume the ROS workspace directory is ``~/ros2_ws`` and 
the source code for this module has been cloned under ``~/ros2_ws/src/slicer_ros2_module``.

You will first need to source the ROS setup script for ROS 2 (foxy or galactic):

.. code-block::bash

    source /opt/ros/galactic/setup.bash

Then build the module using `colcon` while providing the path to your Slicer build directory:

.. code-block::bash

    cd ~/ros2_ws
    colcon build --cmake-args -DSlicer_DIR:PATH=/home/your_user_name_here/something_something/Slicer-SuperBuild-Debug/Slicer-build

.. note:: 

    The ``-DSlicer_DIR...`` option is only needed for the first ``colcon build``.

If you prefer to use CMake (``ccmake``) instead of passing the ``Slicer_DIR`` on the colcon command line, 
you can run ``colcon build`` once and then run ``ccmake`` on the ``slicer_ros2_module`` build directory.  

.. warning::

    You should see the following error messages if the ``Slicer_DIR`` is not set properly (or if Slicer has not been built from scratch):

    .. code-block::

        Could not find a package configuration file provided by "Slicer" with any
        of the following names:

        SlicerConfig.cmake
        slicer-config.cmake

    If you see this message, run CMake on the build directory for 
    ``slicer_ros2_module`` using ``ccmake ~/ros2_ws/build/slicer_ros2_module``.  
    In CMake, set ``Slicer_DIR`` to point to your Slicer build directory 
    then hit ``c`` to configure until you can hit ``g`` to generate the makefiles.  
    Then try to ``colcon build`` again (after ``cd ~/ros2_ws``).
