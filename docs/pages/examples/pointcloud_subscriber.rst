PointCloud Subscriber
=====================

This example demonstrates how to stream a simulated point cloud from a terminal
node using a ROS 2 topic, and visualize it dynamically in 3D Slicer.

Running the Point Cloud Publisher
----------------------------------

We have provided a demo script, ``demo_point_cloud.py``, which publishes a 10x20
point cloud simulating a 2D wavelet at 10Hz.

To run the publisher, first make sure you have sourced your ROS 2 environment and
built the workspace. Then run:

.. code-block:: bash

   # Source the workspace setup
   source ~/ros2_ws/install/setup.bash

   # Run the demo publisher script
   ros2 run slicer_ros2_module demo_point_cloud.py

The publisher will start generating the simulated point cloud and publishing it
over the ROS topic ``/simulated_point_cloud``.

Visualizing in 3D Slicer
--------------------------

Open the 3D Slicer application using the recommended launcher
``ros2 run slicer_ros2_module slicer``.

You can set up the subscriber in one of two ways:

**Option A – run the installed example at startup** using the ``slicer`` launcher's
``--python-script`` argument (the stub starts the installed example after Slicer loads):

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --python-script "$(ros2 pkg prefix slicer_ros2_module)/share/slicer_ros2_module/examples/point_cloud_subscriber.py"

**Option B – use the Python Interactor** (Ctrl + 3):

.. code-block:: python

   from slicerros2.examples import point_cloud_subscriber
   demo = point_cloud_subscriber.run()

Stop the example and release its timer/subscription and scene nodes with
``point_cloud_subscriber.cleanup()``.

The implementation below is included directly from the installed package's source:

.. literalinclude:: ../../../python/slicerros2/examples/point_cloud_subscriber.py
   :language: python
   :pyobject: run

Once run, you will see a 10x20 grid of points in the 3D view representing a dynamic
2D wavelet propagating outwards from the center.
