=======================
Images and Point Clouds
=======================

High-volume sensor data, such as images and point clouds, require efficient handling to avoid performance bottlenecks in Slicer. SlicerROS2 provides native C++ data bridges that perform high-speed conversion between ROS 2 messages and Slicer's internal data structures.

Unlike basic topics that return simple data types (like strings or matrices), image and point cloud subscribers are designed to bridge data directly into Slicer's visualization nodes (Volumes and Models).

.. list-table:: Sensor Data Bridges
   :widths: 30 40 30
   :header-rows: 1

   * - Slicer type
     - ROS type
     - SlicerROS2 "name"
   * - vtkImageData
     - sensor_msgs/msg/Image
     - Image
   * - vtkPolyData
     - sensor_msgs/msg/PointCloud2
     - PolyData

Explicit Bridging
=================

To avoid scene pollution, SlicerROS2 uses an **Explicit Bridging** model for sensor data. By default, a subscriber will receive data but will not visualize it. To visualize the data, you must link the subscriber node to a "Target Node" in the Slicer scene.

* For **Image** subscribers, the target node should be a ``vtkMRMLScalarVolumeNode``.
* For **PolyData** subscribers, the target node should be a ``vtkMRMLModelNode``.

When a target node is linked, the SlicerROS2 logic automatically updates that node's contents whenever a new ROS 2 message arrives, using high-performance C++ memory copying.

Images
======

Image subscribers support multiple encodings including ``mono8``, ``rgb8``, ``rgba8``, ``mono16``, and ``32FC1``.

.. tabs::

   .. tab:: **Python**

      .. code-block:: python

         # 1. Create the subscriber
         sub = rosNode.CreateAndAddSubscriberNode('Image', '/camera/image_raw')

         # 2. Create a Volume node to display the image
         volumeNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLScalarVolumeNode', 'LiveCamera')

         # 3. Link them together
         sub.SetTargetNodeID(volumeNode.GetID())

   .. tab:: **C++**

      .. code-block:: C++

         // 1. Create the subscriber
         auto sub = rosNode->CreateAndAddSubscriberNode("Image", "/camera/image_raw");

         // 2. Create a Volume node
         auto volumeNode = vtkMRMLScalarVolumeNode::SafeDownCast(
           scene->AddNewNodeByClass("vtkMRMLScalarVolumeNode", "LiveCamera"));

         // 3. Link them
         sub->SetTargetNodeID(volumeNode->GetID());

Point Clouds
============

Point cloud subscribers convert ``sensor_msgs/PointCloud2`` messages into ``vtkPolyData``. The bridge automatically generates vertex cells, making the point cloud immediately renderable in Slicer's 3D views.

.. tabs::

   .. tab:: **Python**

      .. code-block:: python

         # 1. Create the subscriber
         sub = rosNode.CreateAndAddSubscriberNode('PolyData', '/lidar_points')

         # 2. Create a Model node to display the points
         modelNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'LiveLidar')
         modelNode.CreateDefaultDisplayNodes()
         
         # Optional: Set point size for better visibility
         modelNode.GetDisplayNode().SetPointSize(3.0)

         # 3. Link them
         sub.SetTargetNodeID(modelNode.GetID())

   .. tab:: **C++**

      .. code-block:: C++

         // 1. Create the subscriber
         auto sub = rosNode->CreateAndAddSubscriberNode("PolyData", "/lidar_points");

         // 2. Create a Model node
         auto modelNode = vtkMRMLModelNode::SafeDownCast(
           scene->AddNewNodeByClass("vtkMRMLModelNode", "LiveLidar"));
         modelNode->CreateDefaultDisplayNodes();

         // 3. Link them
         sub->SetTargetNodeID(modelNode->GetID());
