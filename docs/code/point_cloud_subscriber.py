import slicer

# 1. Create a Model node to display the point cloud
modelNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'SimulatedPointCloud')
modelNode.CreateDefaultDisplayNodes()

# Set point size and color for better visibility in 3D view
displayNode = modelNode.GetDisplayNode()
displayNode.SetPointSize(5.0)
displayNode.SetColor(0.2, 0.6, 1.0) # Sleek blue color

# 2. Retrieve the default ROS 2 node from Slicer
rosLogic = slicer.util.getModuleLogic('ROS2')
rosNode = rosLogic.GetDefaultROS2Node()

# 3. Create the subscriber for PointCloud2 (using 'PolyData' bridge type)
sub = rosNode.CreateAndAddSubscriberNode('PolyData', '/simulated_point_cloud')

# 4. Link the subscriber node to the Model node
sub.SetTargetNodeID(modelNode.GetID())

print("Point cloud subscriber successfully initialized!")
# [end: code]

# Open the ROS 2 module UI so the user can interact with the scene immediately.
# This line is intentionally outside the doc tags and will not appear in
# the documentation.
slicer.util.selectModule('ROS2')
