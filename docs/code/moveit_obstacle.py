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
FIXED_FRAME        = "world"                 # MoveIt fixed frame (TF2 parent of obstacle)
TF_PREFIX          = ""                      # tf prefix, leave empty if none
JOINT_STATES_TOPIC = "/joint_states"
PLANNING_GROUP     = "ur_manipulator"        # MoveIt planning group name
MOVE_GROUP_EXISTS  = True                    # set False to skip MoveIt IK setup
# [end: config]

# [doc: setup]
# ---------------------------------------------------------------------------
# Set up the robot and MoveIt
# ---------------------------------------------------------------------------

# 1. Get the default ROS 2 node (created automatically by the ROS 2 module)
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
# Create the obstacle geometry centred at the origin
# ---------------------------------------------------------------------------

# Build a 200 mm × 200 mm × 200 mm box centred at (0, 0, 0).
# The actual position in the scene is controlled entirely by the MRML
# transform below – nothing is baked into the mesh vertices.
cube = vtk.vtkCubeSource()
cube.SetXLength(200.0)
cube.SetYLength(200.0)
cube.SetZLength(200.0)
# NOTE: SetCenter is intentionally omitted – cube stays at the origin.
cube.Update()

# Wrap it in a vtkMRMLModelNode so it appears in the Slicer scene.
# The node *name* becomes the CollisionObject id in MoveIt and the
# child_frame_id in the TF2 broadcast – use a unique, descriptive name.
obstacleNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'MoveItObstacle')
obstacleNode.SetAndObservePolyData(cube.GetOutput())
obstacleNode.CreateDefaultDisplayNodes()

# Style it so it is clearly visible in the 3D view
dispNode = obstacleNode.GetDisplayNode()
dispNode.SetColor(1.0, 0.5, 0.0)   # orange
dispNode.SetOpacity(0.6)

print("Obstacle node created:", obstacleNode.GetName())
# [end: geometry]

# [doc: transform]
# ---------------------------------------------------------------------------
# Position the obstacle via a MRML LinearTransformNode
# ---------------------------------------------------------------------------
# This transform is the "registration" – it expresses where the obstacle
# (anatomy) is relative to the world/robot frame.  Move it interactively
# in the 3D view or write to it programmatically from a registration algorithm.

obstacleTransform = slicer.mrmlScene.AddNewNodeByClass(
    'vtkMRMLLinearTransformNode', 'ObstacleTransform'
)

# Show the interactive gizmo in the 3D view so the user can drag the obstacle
transformDisplayNode = slicer.mrmlScene.AddNewNodeByClass(
    'vtkMRMLTransformDisplayNode', 'ObstacleTransform_Display'
)
transformDisplayNode.SetVisibility(True)
transformDisplayNode.SetEditorVisibility(True)   # show the 3D gizmo
obstacleTransform.SetAndObserveDisplayNodeID(transformDisplayNode.GetID())

# Set the initial placement: 400 mm along X, 300 mm up along Z.
# (Same position as the old hard-coded cube centre, now as a moveable transform.)
m = vtk.vtkMatrix4x4()
m.SetElement(0, 3, 400.0)   # X translation in mm
m.SetElement(2, 3, 300.0)   # Z translation in mm
obstacleTransform.SetMatrixTransformToParent(m)

# Attach the obstacle model to this transform so it moves in the 3D view
obstacleNode.SetAndObserveTransformNodeID(obstacleTransform.GetID())

print(f"Obstacle positioned at (400, 0, 300) mm via '{obstacleTransform.GetName()}'.")
print("Drag the gizmo in the 3D view to reposition the obstacle.")
# [end: transform]

# [doc: publish]
# ---------------------------------------------------------------------------
# Register the obstacle with MoveIt and start broadcasting its transform
# ---------------------------------------------------------------------------
# AddMoveItObstacleWithTransform does three things in one call:
#   1. Publishes a CollisionObject whose header.frame_id = "MoveItObstacle"
#      (the obstacle's node name).  MoveIt resolves the position via TF2.
#   2. Creates a TF2 broadcaster node: parent="world", child="MoveItObstacle".
#   3. Observes ObstacleTransform so every change is re-broadcast over /tf
#      automatically and in real time.

broadcaster, observerTag = motionLogic.AddMoveItObstacleWithTransform(
    obstacleNode,
    obstacleTransform,
    FIXED_FRAME,       # parent TF2 frame, e.g. "world"
    robotNode          # used to locate the robot root transform and ROS 2 node
)
if broadcaster is None:
    raise RuntimeError("Failed to publish obstacle to MoveIt.")

print("Obstacle published to MoveIt planning scene.")
print(f"TF2 broadcasting: '{FIXED_FRAME}' -> '{obstacleNode.GetName()}'")
print("Drag 'ObstacleTransform' in the 3D view – MoveIt planning scene updates live.")
# [end: publish]

# Open the ROS2 Motion Control module UI so the user can interact with the
# planning scene immediately.  This line is intentionally outside the doc tags
# and will not appear in the documentation.
slicer.util.selectModule('ROS2MotionControl')
