Launching Slicer
================

After building and sourcing the workspace, start Slicer with the wrapper script:

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash
   ros2 run slicer_ros2_module slicer

The wrapper launches the Slicer executable through the ROS 2 launch system and
configures the module and library paths needed by SlicerROS2. This is the
recommended way to start Slicer for normal use.

Wrapper vs Launch File
----------------------

The ``ros2 run slicer_ros2_module slicer`` command is a convenience wrapper
around the package launch file:

.. code-block:: bash

   ros2 launch slicer_ros2_module slicer.launch.py

Both commands start Slicer with the same ROS 2 workspace environment, module
paths, extension paths, and library paths.  The wrapper is easier for day-to-day
interactive use because Slicer arguments can be written directly after the
command.  The launch file is the lower-level ROS 2 entry point and is useful for
automation, tests, and launch compositions that need to include Slicer alongside
other ROS 2 nodes.

For normal use, prefer the wrapper:

.. code-block:: bash

   ros2 run slicer_ros2_module slicer

Use the launch file when you specifically need ROS 2 launch-system behavior:

.. code-block:: bash

   ros2 launch slicer_ros2_module slicer.launch.py slicer_args:="--no-splash"

Passing Arguments
-----------------

Arguments after the wrapper command are passed to Slicer:

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --no-splash --homedir /tmp/slicer-ros2

Python scripts can be run at startup:

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --python-script my_script.py

Extension Paths
---------------

Extensions installed with the SlicerROS2 extension manager are registered in the
workspace install tree. The launcher reads these paths so the extensions are
available when Slicer starts.

.. code-block:: bash

   ros2 run slicer_ros2_module manage_extensions.py

Directly starting the Slicer executable can work for unrelated Slicer tasks, but
it bypasses the ROS 2 workspace environment and may prevent SlicerROS2 or
managed extensions from loading correctly.
