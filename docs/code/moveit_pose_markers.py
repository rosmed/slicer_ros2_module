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

# Lower the marker path from the bent demo seed pose.
MARKER_Z_OFFSET_MM = -200.0
PATH_X_MM = 100.0
PATH_Y_MM = 100.0

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

rootLink, tipLink = rootTipLinks[:2]
rootTransform = motionLogic.findRobotTransforms(rootLink, goal=True)


# ---------------------------------------------------------------------------
# Build a Python list of built-in 3D pose markers
# ---------------------------------------------------------------------------
def wait_for_joint_state(joint_names, timeout_sec=5.0):
    deadline = time.time() + float(timeout_sec)
    while time.time() < deadline:
        slicer.app.processEvents()
        rosLogic.Spin()
        joint_values = motionLogic.GetCurrentJointState(joint_names)
        if joint_values and len(joint_values) == len(joint_names):
            return list(joint_values)
        time.sleep(0.05)
    return []


def create_marker_visuals(markers, radius_mm=6.0):
    if not markers:
        return None

    sphere = vtk.vtkSphereSource()
    sphere.SetRadius(float(radius_mm))
    sphere.SetThetaResolution(20)
    sphere.SetPhiResolution(20)
    sphere.Update()

    visualNodes = []
    for i, marker in enumerate(markers):
        visualNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", f"{marker.GetName()}_Visual")
        visualNode.SetAndObservePolyData(sphere.GetOutput())

        displayNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelDisplayNode", f"{marker.GetName()}_Visual_Display")
        displayNode.SetColor(0.0, 0.8, 0.3)
        displayNode.SetOpacity(0.9)
        displayNode.SetVisibility3D(True)
        displayNode.SetVisibility2D(True)
        visualNode.SetAndObserveDisplayNodeID(displayNode.GetID())

        # Parent visual geometry to the marker transform so it follows marker edits.
        visualNode.SetAndObserveTransformNodeID(marker.GetID())
        visualNodes.append(visualNode)

    return visualNodes


jointNames = list(robotNode.GetJoints())
liveJointValues = wait_for_joint_state(jointNames)
if not liveJointValues:
    raise RuntimeError(
        "Did not receive a valid /joint_states message within timeout. "
        "Verify the robot driver or state publisher is running, then retry."
    )

startJointValues = []
startJointNamesForPlanning = []
startStateSource = "MoveIt current state (from live /joint_states)"
markerAnchorJointValues = list(liveJointValues)
markerAnchorSource = "current tip pose"

anchorPoseRoot = vtk.vtkMatrix4x4()
if robotNode.ComputeKDLFK(markerAnchorJointValues, anchorPoseRoot, tipLink) is None:
    raise RuntimeError("Failed to compute FK for marker anchor joint state.")

rootWorld = vtk.vtkMatrix4x4()
rootTransform.GetMatrixTransformToWorld(rootWorld)

anchorPose = vtk.vtkMatrix4x4()
vtk.vtkMatrix4x4.Multiply4x4(rootWorld, anchorPoseRoot, anchorPose)


def pose_with_offset(sourceMatrix, dxMm, dyMm, dzMm):
    matrix = vtk.vtkMatrix4x4()
    matrix.DeepCopy(sourceMatrix)
    matrix.SetElement(0, 3, matrix.GetElement(0, 3) + dxMm)
    matrix.SetElement(1, 3, matrix.GetElement(1, 3) + dyMm)
    matrix.SetElement(2, 3, matrix.GetElement(2, 3) + dzMm)
    return matrix


loweredAnchorPose = pose_with_offset(anchorPose, 0.0, 0.0, MARKER_Z_OFFSET_MM)
poseMarkers = [
    motionLogic.CreatePoseMarker("PathPose_0", anchorPose),
    motionLogic.CreatePoseMarker("PathPose_1", loweredAnchorPose),
    motionLogic.CreatePoseMarker("PathPose_2", pose_with_offset(loweredAnchorPose, PATH_X_MM, 0.0, 0.0)),
    motionLogic.CreatePoseMarker("PathPose_3", pose_with_offset(loweredAnchorPose, PATH_X_MM, PATH_Y_MM, 0.0)),
    motionLogic.CreatePoseMarker("PathPose_4", pose_with_offset(loweredAnchorPose, 0.0, PATH_Y_MM, 0.0)),
]

markerVisualNodes = create_marker_visuals(poseMarkers)

print("Created pose markers:")
for marker in poseMarkers:
    print("  ", marker.GetName())
if markerVisualNodes:
    print(f"Created {len(markerVisualNodes)} marker visuals (spheres attached to transforms).")
print(f"Start state source: {startStateSource}")
print(f"Path starts at {markerAnchorSource}, then lowers before XY motion.")
print(f"PathPose_1 is {abs(MARKER_Z_OFFSET_MM):.0f} mm below PathPose_0.")
print(
    "Pose markers are visible; 3D editor widgets are hidden. "
    "Set marker.GetDisplayNode().SetEditorVisibility(True) to edit one interactively."
)


# ---------------------------------------------------------------------------
# Plan a Cartesian trajectory through all marker poses
# ---------------------------------------------------------------------------
trajectory = motionLogic.PlanMoveItCartesianTrajectoryFromPoseMarkers(
    motionControlNode=motionControlNode,
    groupName=PLANNING_GROUP,
    poseMarkers=poseMarkers,
    relativeToNode=rootTransform,
    robotNode=robotNode,
    eefStepMeters=0.01,
    jumpThreshold=0.0,
    avoidCollisions=True,
    velocityScaling=0.5,
    accelerationScaling=0.5,
    planningTimeSec=5.0,
    startJointNames=startJointNamesForPlanning,
    startJointValues=startJointValues,
)
fraction = motionControlNode.GetLastCartesianPathFraction()
if trajectory is None:
    print(f"MoveIt Cartesian path fraction: {fraction:.3f}")
    print("MoveIt did not return a usable Cartesian trajectory.")
    print("Try enabling one marker's editor widget and moving the marker closer to PathPose_0.")
else:
    pointCount = len(trajectory.GetJointTrajectory().GetPoints())
    print(f"MoveIt Cartesian path fraction: {fraction:.3f}")
    print(f"Trajectory points: {pointCount}")
    if fraction < 0.99:
        print("Warning: MoveIt planned only a partial Cartesian path.")

    slicer.util.selectModule("ROS2MotionControl")
    motionWidget = slicer.util.getModuleWidget("ROS2MotionControl")

    if motionWidget.loadPlannedTrajectory(trajectory, enableExecute=True):
        print("Trajectory loaded into the Motion Control GUI.")
        print("Use Preview, the scrubber, or Execute from the Motion Control module.")
