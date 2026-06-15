import slicer
import vtk
import time

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
ROBOT_NAME = "ur5"
URDF_NODE_NAME = "robot_state_publisher"
URDF_PARAM_NAME = "robot_description"
FIXED_FRAME = "world"
TF_PREFIX = ""
JOINT_STATES_TOPIC = "/joint_states"
PLANNING_GROUP = "ur_manipulator"

TOOL_LINK = "tool0"
NEEDLE_LENGTH_MM = 100.0
NEEDLE_RADIUS_MM = 1.0
NEEDLE_OBJECT_NAME = "SlicerNeedle"


# ---------------------------------------------------------------------------
# Robot and MotionControl setup
# ---------------------------------------------------------------------------
rosLogic = slicer.util.getModuleLogic("ROS2")
rosNode = rosLogic.GetDefaultROS2Node()

robotNode = rosNode.CreateAndAddRobotNode(
    ROBOT_NAME, URDF_NODE_NAME, URDF_PARAM_NAME, FIXED_FRAME, TF_PREFIX
)
if robotNode is None:
    raise RuntimeError(f"Robot node '{ROBOT_NAME}' was not created.")

rootTipLinks = None
for _ in range(50):
    slicer.app.processEvents()
    rootTipLinks = robotNode.FindRootAndTipLinks()
    if rootTipLinks and len(rootTipLinks) >= 2:
        break
    time.sleep(0.2)

if not rootTipLinks or len(rootTipLinks) < 2:
    raise RuntimeError(
        "Could not resolve robot root/tip links within timeout. "
        "Verify robot_state_publisher and TF are running."
    )

motionLogic = slicer.util.getModuleLogic("ROS2MotionControl")
paramNode = motionLogic.getParameterNode()
paramNode.robotNodeID = robotNode.GetID()
paramNode.jointStateTopic = JOINT_STATES_TOPIC
paramNode.planningGroup = PLANNING_GROUP
paramNode.moveGroupExists = True

if not motionLogic.SetupRobotForMotionControl(paramNode):
    raise RuntimeError("Failed to set up robot for motion control.")

motionControlNode = slicer.mrmlScene.GetNodeByID(paramNode.motionControlNodeID)
if motionControlNode is None:
    raise RuntimeError("Motion control node was not created.")


# ---------------------------------------------------------------------------
# Create the obstacle geometry centred at the origin
# ---------------------------------------------------------------------------
# This is the same pattern used in moveit_obstacle.py.  The actual position is
# controlled by the MRML transform below.
cube = vtk.vtkCubeSource()
cube.SetXLength(200.0)
cube.SetYLength(200.0)
cube.SetZLength(200.0)
cube.Update()

obstacleNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", "MoveItObstacle")
obstacleNode.SetAndObservePolyData(cube.GetOutput())
obstacleNode.CreateDefaultDisplayNodes()
obstacleNode.GetDisplayNode().SetColor(1.0, 0.5, 0.0)
obstacleNode.GetDisplayNode().SetOpacity(0.6)

# ---------------------------------------------------------------------------
# Position the obstacle via a MRML LinearTransformNode
# ---------------------------------------------------------------------------
obstacleTransform = slicer.mrmlScene.AddNewNodeByClass(
    "vtkMRMLLinearTransformNode", "ObstacleTransform"
)
obstacleMatrix = vtk.vtkMatrix4x4()
obstacleMatrix.SetElement(0, 3, 400.0)
obstacleMatrix.SetElement(2, 3, 300.0)
obstacleTransform.SetMatrixTransformToParent(obstacleMatrix)
obstacleNode.SetAndObserveTransformNodeID(obstacleTransform.GetID())

# ---------------------------------------------------------------------------
# Register the obstacle with MoveIt and start broadcasting its transform
# ---------------------------------------------------------------------------
broadcaster, observerTag = motionLogic.AddMoveItObstacleWithTransform(
    obstacleNode,
    obstacleTransform,
    FIXED_FRAME,
    robotNode,
)
if broadcaster is None:
    raise RuntimeError("Failed to publish obstacle to MoveIt.")


# [doc: needle]
# ---------------------------------------------------------------------------
# Add a runtime 100 mm needle tool attached to the UR tool frame.
# ---------------------------------------------------------------------------

# The cylinder mesh is authored in the TOOL_LINK frame.  Its base is at the
# tool origin and its tip is 100 mm along +Z.  MoveIt receives this mesh as an
# AttachedCollisionObject, so the needle participates in robot collision checks.
cylinder = vtk.vtkCylinderSource()
cylinder.SetRadius(NEEDLE_RADIUS_MM)
cylinder.SetHeight(NEEDLE_LENGTH_MM)
cylinder.SetCenter(0.0, NEEDLE_LENGTH_MM / 2.0, 0.0)
cylinder.SetResolution(24)
cylinder.CappingOn()
cylinder.Update()

alignCylinderToZ = vtk.vtkTransform()
alignCylinderToZ.RotateX(90.0)

needleFilter = vtk.vtkTransformPolyDataFilter()
needleFilter.SetInputConnection(cylinder.GetOutputPort())
needleFilter.SetTransform(alignCylinderToZ)
needleFilter.Update()

needleNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", NEEDLE_OBJECT_NAME)
needleNode.SetAndObservePolyData(needleFilter.GetOutput())
needleNode.CreateDefaultDisplayNodes()
needleNode.GetDisplayNode().SetColor(0.1, 0.7, 1.0)
needleNode.GetDisplayNode().SetOpacity(0.85)

# Define a MoveIt object subframe named "tip" at the distal needle point.
needleTipPoseInTool = motionLogic.CreateToolTipPose(0.0, 0.0, NEEDLE_LENGTH_MM)
needleTipLink = motionLogic.AddMoveItAttachedTool(
    modelNode=needleNode,
    linkName=TOOL_LINK,
    touchLinks=[TOOL_LINK, "flange", "wrist_3_link"],
    tipSubframeName="tip",
    tipPose=needleTipPoseInTool,
    robotNode=robotNode,
)
if not needleTipLink:
    raise RuntimeError("Failed to attach needle tool to MoveIt.")

print(f"Attached needle tool.  MoveIt tip frame: {needleTipLink}")
# [end: needle]


slicer.util.selectModule("ROS2MotionControl")
motionWidget = slicer.util.getModuleWidget("ROS2MotionControl")
if hasattr(motionWidget, "refreshEndEffectorLinkComboBox"):
    motionWidget.refreshEndEffectorLinkComboBox(selectLink=needleTipLink)

print("The needle model is parented to the UR tool link in Slicer.")
print("The Motion Control end-effector selector is set to the needle tip.")
