""""""""
Examples
""""""""

This page provides various step-by-step examples demonstrating how to use SlicerROS2 to connect 3D Slicer with ROS 2 workflows.

=====================
PointCloud subscriber
=====================

This example demonstrates how to stream a simulated point cloud from a terminal node using a ROS 2 topic, and visualize it dynamically in 3D Slicer.

Running the Point Cloud Publisher
=================================

We have provided a demo script, ``demo_point_cloud.py``, which publishes a 10x20 point cloud simulating a 2D wavelet at 10Hz.

To run the publisher, first make sure you have sourced your ROS 2 environment and built the workspace. Then run:

.. code-block:: bash

   # Source the workspace setup
   source ~/ros2_ws/install/setup.bash

   # Run the demo publisher script
   ros2 run slicer_ros2_module demo_point_cloud.py

The publisher will start generating the simulated point cloud and publishing it over the ROS topic ``/simulated_point_cloud``.

Visualizing in 3D Slicer
========================

Open the 3D Slicer application (launched using the recommended launcher ``ros2 launch slicer_ros2_module slicer.launch.py``).

Open Slicer's Python Interactor (Ctrl + 3 or through the **Developer Tools** menu) and copy/paste the following code to setup a subscriber, create a 3D model node, and link them:

.. code-block:: python

   import slicer

   # 1. Create a Model node to display the point cloud
   modelNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'SimulatedPointCloud')
   modelNode.CreateDefaultDisplayNodes()

   # Set point size and color for better visibility in 3D view
   displayNode = modelNode.GetDisplayNode()
   displayNode.SetPointSize(5.0)
   displayNode.SetColor(0.2, 0.6, 1.0) # Sleek blue color

   # 2. Retrieve the default ROS 2 Node from Slicer
   rosLogic = slicer.util.getModuleLogic('ROS2')
   rosNode = rosLogic.GetDefaultROS2Node()

   # 3. Create the subscriber for PointCloud2 (using 'PolyData' bridge type)
   sub = rosNode.CreateAndAddSubscriberNode('PolyData', '/simulated_point_cloud')

   # 4. Link the subscriber node to the Model node
   sub.SetTargetNodeID(modelNode.GetID())

   print("Point cloud subscriber successfully initialized!")

Once run, you will see a 10x20 grid of points in the 3D view representing a dynamic 2D wavelet propagating outwards from the center.

====================
PointCloud publisher
====================

This example demonstrates how to create a static point cloud using 3D Slicer's built-in features and publish it over a ROS 2 topic to be visualized in RViz.

Creating and Publishing the Point Cloud in 3D Slicer
====================================================

Open 3D Slicer's Python Interactor (**Ctrl + 3** or through the **Developer Tools** menu). Copy and paste the following Python script to create a static sphere-shaped point cloud using VTK/Slicer, and publish it using the native **SlicerROS2** PolyData publisher:

.. code-block:: python

   import slicer
   import vtk

   # 1. Create a sphere using Slicer's built-in VTK library to serve as a static point cloud
   sphere = vtk.vtkSphereSource()
   sphere.SetRadius(15.0)
   sphere.SetThetaResolution(20)
   sphere.SetPhiResolution(20)
   sphere.Update()

   # Create a Model node in the Slicer scene to visualize it locally first
   modelNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'SlicerStaticPointCloud')
   modelNode.SetAndObservePolyData(sphere.GetOutput())
   modelNode.CreateDefaultDisplayNodes()

   # 2. Retrieve Slicer's default ROS 2 Node
   rosLogic = slicer.util.getModuleLogic('ROS2')
   rosNode = rosLogic.GetDefaultROS2Node()

   # 3. Create the native PointCloud2 publisher (using 'PolyData' bridge type)
   # We publish onto the '/slicer_point_cloud' topic
   pub = rosNode.CreateAndAddPublisherNode('PolyData', '/slicer_point_cloud')

   # 4. Set the frame ID natively on the publisher node!
   pub.SetFrameId('world')

   # 5. Publish the point cloud
   pub.Publish(modelNode.GetPolyData())

   print("Successfully published static point cloud of sphere via native SlicerROS2 publisher!")

Once executed, Slicer will create a local model displaying the sphere, and publish the points to the ROS 2 topic ``/slicer_point_cloud`` with the ``frame_id`` set to ``world``.

Visualizing in RViz
===================

To visualize this published point cloud in RViz:

1. **Open a terminal**, source the ROS 2 setup script, and launch RViz:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      rviz2

2. **Configure RViz Display**:
   - In the **Global Options** panel, set the **Fixed Frame** to ``world``.
   - Click the **Add** button at the bottom left of the window.
   - Switch to the **By topic** tab, expand ``/slicer_point_cloud``, select **PointCloud2**, and click **OK**.
   - (Optional) In the newly added PointCloud2 display properties, increase the **Size (m)** parameter to ``0.2`` or larger to render the sphere points clearly in the 3D grid.
