"""Import this example and call run() explicitly inside Slicer."""

_state = None


def run():
    """Start the demo and return its ROS/MRML objects."""
    from types import SimpleNamespace
    global _state
    cleanup()
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


    # Open the ROS 2 module UI so the user can interact with the scene immediately.
    slicer.util.selectModule('ROS2')

    sceneObserverTag = slicer.mrmlScene.AddObserver(slicer.mrmlScene.StartCloseEvent, lambda *args: cleanup())
    _state = SimpleNamespace(**locals())
    return _state


def cleanup():
    """Stop the demo and release the nodes it created."""
    import slicer
    global _state
    if _state is None:
        return
    slicer.mrmlScene.RemoveObserver(_state.sceneObserverTag)
    if _state.rosNode.GetScene() is not None:
        _state.rosNode.RemoveAndDeleteSubscriberNode("/simulated_point_cloud")
    display = _state.modelNode.GetDisplayNode()
    if _state.modelNode.GetScene() is not None:
        slicer.mrmlScene.RemoveNode(_state.modelNode)
    if display is not None and display.GetScene() is not None:
        slicer.mrmlScene.RemoveNode(display)
    _state = None


if __name__ == "__main__":
    run()
