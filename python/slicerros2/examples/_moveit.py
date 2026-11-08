"""Shared implementation for the documented MoveIt examples."""


def setup_robot(robot_name="ur5", parameter_node="robot_state_publisher",
                parameter="robot_description", fixed_frame="world", tf_prefix="",
                joint_states="/joint_states", group="ur_manipulator", move_group=True):
    import time
    from types import SimpleNamespace
    import slicer
    from slicerros2.robots import get_ros_node, load_robot

    # [doc: setup]
    rosLogic = slicer.util.getModuleLogic("ROS2")
    rosNode = get_ros_node()
    robotNode = load_robot(robot_name, parameter_node, parameter=parameter,
                           fixed_frame=fixed_frame, tf_prefix=tf_prefix, ros_node=rosNode)
    deadline = time.monotonic() + 10.0
    rootTipLinks = robotNode.FindRootAndTipLinks()
    while not rootTipLinks and time.monotonic() < deadline:
        slicer.app.processEvents()
        time.sleep(.02)
        rootTipLinks = robotNode.FindRootAndTipLinks()
    if not rootTipLinks or len(rootTipLinks) < 2:
        raise RuntimeError("Robot root/tip links unavailable. Check robot_state_publisher and TF.")
    motionLogic = slicer.util.getModuleLogic("ROS2MotionControl")
    paramNode = motionLogic.getParameterNode()
    paramNode.robotNodeID = robotNode.GetID()
    paramNode.jointStateTopic = joint_states
    paramNode.planningGroup = group
    paramNode.moveGroupExists = move_group
    if not motionLogic.SetupRobotForMotionControl(paramNode):
        raise RuntimeError("Failed to set up robot for motion control.")
    motionControlNode = slicer.mrmlScene.GetNodeByID(paramNode.motionControlNodeID)
    if motionControlNode is None:
        raise RuntimeError("Motion control node was not created.")
    rootLink, tipLink = rootTipLinks[:2]
    rootTransform = motionLogic.findRobotTransforms(rootLink, goal=True)
    # [end: setup]
    return SimpleNamespace(**locals())


def add_obstacle(motionLogic, robotNode, fixed_frame="world"):
    import slicer
    import vtk
    from types import SimpleNamespace

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
        fixed_frame,       # parent TF2 frame, e.g. "world"
        robotNode          # used to locate the robot root transform and ROS 2 node
    )
    if broadcaster is None:
        raise RuntimeError("Failed to publish obstacle to MoveIt.")

    print("Obstacle published to MoveIt planning scene.")
    print(f"TF2 broadcasting: '{fixed_frame}' -> '{obstacleNode.GetName()}'")
    print("Drag 'ObstacleTransform' in the 3D view – MoveIt planning scene updates live.")
    # [end: publish]

    return SimpleNamespace(**locals())
