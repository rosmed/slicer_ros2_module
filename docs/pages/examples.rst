Examples
========

This section provides step-by-step examples demonstrating how to use SlicerROS2
to connect 3D Slicer with ROS 2 workflows.

All complete demos are installed under ``slicerros2.examples``. Importing a
module does not start it; call ``run()`` explicitly. Returned demo objects keep
their timers and scene objects available for interactive use. Documentation
includes these package sources directly; there is no separate docs/code copy.
Reusable helpers are documented under :doc:`/pages/utilities`. Small startup stubs in
``share/slicer_ros2_module/examples`` provide ``--python-script`` entry points;
these stubs contain no example implementation.

Running examples
----------------

From a sourced ROS workspace, start Slicer and run an installed startup stub:

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --python-script "$(ros2 pkg prefix slicer_ros2_module)/share/slicer_ros2_module/examples/point_cloud_publisher.py"

Each stub only imports its module and calls ``run()``. Replace the filename with
any example listed below. The implementation lives in the Python package, so
there is no duplicate demo code to maintain.

Inside an already running Slicer, use the Python Interactor (Ctrl + 3):

.. code-block:: python

   from slicerros2.examples import point_cloud_publisher
   demo = point_cloud_publisher.run()
   # Stop this example when finished:
   point_cloud_publisher.cleanup()

The same import and call can be supplied directly at startup:

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --python-code "from slicerros2.examples import point_cloud_publisher; demo = point_cloud_publisher.run()"

For the existing ROS launch-file interface and argument forwarding, see
:doc:`/pages/launching-slicer`. No new launcher arguments are required.

Examples
--------

For background on the APIs used by these examples, see
:doc:`/pages/nodes/topics`, :doc:`/pages/nodes/images_pointclouds`,
:doc:`/pages/robot-visualization`, and :doc:`/pages/motion-control`.

.. toctree::
   :maxdepth: 2

   examples/pointcloud_subscriber
   examples/pointcloud_publisher
   examples/moveit_obstacle
   examples/moveit_custom_tool
   examples/moveit_pose_markers
   examples/dvrk
