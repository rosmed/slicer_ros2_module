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

**Option A – run the script directly from a terminal** using the ``slicer`` launcher's
``--python-script`` argument (the script executes automatically after Slicer starts):

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --python-script $(ros2 pkg prefix slicer_ros2_module)/share/slicer_ros2_module/docs/code/point_cloud_subscriber.py

**Option B – paste the code** into Slicer's Python Interactor (Ctrl + 3 or through the
**Developer Tools** menu):

.. literalinclude:: ../../code/point_cloud_subscriber.py
   :language: python
   :end-before: [end: code]

Once run, you will see a 10x20 grid of points in the 3D view representing a dynamic
2D wavelet propagating outwards from the center.
