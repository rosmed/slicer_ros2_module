"""Import this example and call run() explicitly inside Slicer."""

# [doc: config]
ROBOT_NAME = "ur5"
URDF_NODE_NAME = "robot_state_publisher"
URDF_PARAM_NAME = "robot_description"
FIXED_FRAME = "world"
TF_PREFIX = ""
JOINT_STATES_TOPIC = "/joint_states"
PLANNING_GROUP = "ur_manipulator"
MARKER_Z_OFFSET_MM = -200.0
PATH_X_MM = 80.0
PATH_Y_MM = 80.0
PLANNING_TIME_SEC = 15.0
# [end: config]

_state = None


def run():
    """Run the demo and return its objects for Python-interactor use."""
    global _state
    from types import SimpleNamespace
    import slicer
    import vtk
    import time

    # ---------------------------------------------------------------------------
    # Configuration
    # ---------------------------------------------------------------------------

    # Lower the marker path from the bent demo seed pose.

    # ---------------------------------------------------------------------------
    # Robot and MotionControl setup
    # ---------------------------------------------------------------------------
    from slicerros2.examples import _moveit
    setup = _moveit.setup_robot(ROBOT_NAME, URDF_NODE_NAME, URDF_PARAM_NAME,
                                FIXED_FRAME, TF_PREFIX, JOINT_STATES_TOPIC, PLANNING_GROUP)
    rosLogic, rosNode, robotNode = setup.rosLogic, setup.rosNode, setup.robotNode
    motionLogic, paramNode = setup.motionLogic, setup.paramNode
    motionControlNode, rootTipLinks = setup.motionControlNode, setup.rootTipLinks
    rootLink, tipLink, rootTransform = setup.rootLink, setup.tipLink, setup.rootTransform



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
        planningTimeSec=PLANNING_TIME_SEC,
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

        if motionWidget.loadPlannedTrajectory(trajectory, enableExecute=True, lockPlanning=True):
            print("Trajectory loaded into the Motion Control GUI.")
            pathNode = motionWidget.addPlannedTrajectoryPolyline()
            if pathNode:
                print(f"End-effector path displayed as: {pathNode.GetName()}")
            print("Plan controls are temporarily locked to avoid overwriting this external trajectory.")
            print("Use Preview, the scrubber, or Execute from the Motion Control module.")

    _state = SimpleNamespace(**locals())
    return _state


if __name__ == "__main__":
    run()
