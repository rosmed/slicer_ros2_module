"""Import this example and call run() explicitly inside Slicer."""

# [doc: config]
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
    obstacle = _moveit.add_obstacle(motionLogic, robotNode, FIXED_FRAME)
    obstacleNode, obstacleTransform = obstacle.obstacleNode, obstacle.obstacleTransform
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

    _state = SimpleNamespace(**locals())
    return _state


if __name__ == "__main__":
    run()
