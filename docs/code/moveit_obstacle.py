import slicer
import vtk
import time

# [doc: config]
# ---------------------------------------------------------------------------
# Configuration – edit these values to match your robot and MoveIt setup.
# ---------------------------------------------------------------------------
ROBOT_NAME         = "ur5"                   # name given to the robot node
URDF_NODE_NAME     = "robot_state_publisher" # ROS 2 node that owns robot_description
URDF_PARAM_NAME    = "robot_description"     # parameter holding the URDF
FIXED_FRAME        = "world"                 # MoveIt fixed frame
TF_PREFIX          = ""                      # tf prefix, leave empty if none
JOINT_STATES_TOPIC = "/joint_states"
PLANNING_GROUP     = "ur_manipulator"        # MoveIt planning group name
MOVE_GROUP_EXISTS  = True                    # set False to skip MoveIt IK setup
# [end: config]

# [doc: setup]
# ---------------------------------------------------------------------------
# Set up the robot and MoveIt
# ---------------------------------------------------------------------------

# 1. Get the default ROS2 node (created automatically by the ROS2 module)
rosLogic = slicer.util.getModuleLogic('ROS2')
rosNode  = rosLogic.GetDefaultROS2Node()

# 2. Create and add the robot node from the URDF parameter.
robotNode = rosNode.CreateAndAddRobotNode(ROBOT_NAME, URDF_NODE_NAME, URDF_PARAM_NAME, FIXED_FRAME, TF_PREFIX)
if robotNode is None:
    raise RuntimeError(f"Robot node '{ROBOT_NAME}' was not created.")

# The URDF is fetched asynchronously. We must pump events until the URDF has been parsed.
for _ in range(50):
    slicer.app.processEvents()
    if robotNode.FindRootAndTipLinks():
        break
    time.sleep(0.2)

# 3. Use the module Parameter Node and setup the motion control logic
#    This keeps the UI perfectly synchronized with our scripted setup!
motionLogic = slicer.util.getModuleLogic('ROS2MotionControl')
paramNode = motionLogic.getParameterNode()

paramNode.robotNodeID = robotNode.GetID()
paramNode.jointStateTopic = JOINT_STATES_TOPIC
paramNode.planningGroup = PLANNING_GROUP
paramNode.moveGroupExists = MOVE_GROUP_EXISTS

if not motionLogic.SetupRobotForMotionControl(paramNode):
    raise RuntimeError("Failed to set up robot for motion control.")

print("Setup complete – UI is synced, and you can now create/publish obstacles.")
# [end: setup]

# [doc: geometry]
# ---------------------------------------------------------------------------
# Create the obstacle geometry
# ---------------------------------------------------------------------------

# Build a box geometry with vtkCubeSource (dimensions in mm)
cube = vtk.vtkCubeSource()
cube.SetXLength(200.0)
cube.SetYLength(200.0)
cube.SetZLength(200.0)
cube.SetCenter(400.0, 0.0, 300.0)
cube.Update()

# Wrap it in a vtkMRMLModelNode so it appears in the Slicer scene
obstacleNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'MoveItObstacle')
obstacleNode.SetAndObservePolyData(cube.GetOutput())
obstacleNode.CreateDefaultDisplayNodes()

# Style it so it is clearly visible in the 3D view
dispNode = obstacleNode.GetDisplayNode()
dispNode.SetColor(1.0, 0.5, 0.0)   # orange
dispNode.SetOpacity(0.6)

print("Obstacle node created:", obstacleNode.GetName())
# [end: geometry]

# [doc: publish]
# ---------------------------------------------------------------------------
# Publish the obstacle to the MoveIt planning scene
# ---------------------------------------------------------------------------

# Mark and publish the model as a MoveIt collision object.
if not motionLogic.AddMoveItObstacle(obstacleNode, FIXED_FRAME, robotNode):
    raise RuntimeError("Failed to publish obstacle to MoveIt.")

print("Obstacle published to MoveIt planning scene.")
# [end: publish]
