import slicer
import vtk

# 1. Build a box geometry with vtkCubeSource (dimensions in mm)
cube = vtk.vtkCubeSource()
cube.SetXLength(200.0)
cube.SetYLength(200.0)
cube.SetZLength(200.0)
cube.SetCenter(400.0, 0.0, 300.0)
cube.Update()

# 2. Wrap it in a vtkMRMLModelNode so it appears in the Slicer scene
obstacleNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'MoveItObstacle')
obstacleNode.SetAndObservePolyData(cube.GetOutput())
obstacleNode.CreateDefaultDisplayNodes()

# Style it so it is clearly visible in the 3D view
dispNode = obstacleNode.GetDisplayNode()
dispNode.SetColor(1.0, 0.5, 0.0)   # orange
dispNode.SetOpacity(0.6)

print("Obstacle node created:", obstacleNode.GetName())

# 3. Get the default SlicerROS2 node
rosLogic = slicer.util.getModuleLogic('ROS2')
rosNode = rosLogic.GetDefaultROS2Node()

# 4. Create the CollisionObject publisher
#    '/planning_scene' is the standard topic that move_group's
#    PlanningSceneMonitor subscribes to for incremental scene diffs.
pub = rosNode.CreateAndAddPublisherNode('CollisionObject', '/planning_scene')

# 5. Point the publisher at our obstacle model and set the reference frame
pub.SetSourceNodeID(obstacleNode.GetID())
pub.SetFrameId('world')   # must match the MoveIt fixed frame

# 6. Publish once – MoveIt adds it to its internal planning scene
pub.Publish()

print("Obstacle published to MoveIt planning scene.")
