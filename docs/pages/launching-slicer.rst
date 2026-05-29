Launching Slicer
================

After building and sourcing the workspace, start Slicer with the wrapper script:

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash
   ros2 run slicer_ros2_module slicer

The wrapper launches the Slicer executable through the ROS 2 launch system and
configures the module and library paths needed by SlicerROS2. This is the
recommended way to start Slicer for normal use.

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
