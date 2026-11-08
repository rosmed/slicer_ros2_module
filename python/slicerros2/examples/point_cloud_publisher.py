"""Import this example and call run() explicitly inside Slicer."""

_state = None


def run():
    """Start the demo and return its ROS/MRML objects."""
    from types import SimpleNamespace
    global _state
    cleanup()
    import slicer
    import vtk
    import qt

    # 1. Build sphere geometry and a transform filter to animate it
    sphere = vtk.vtkSphereSource()
    sphere.SetRadius(15.0)
    sphere.SetThetaResolution(20)
    sphere.SetPhiResolution(20)
    sphere.Update()

    transform = vtk.vtkTransform()
    transformFilter = vtk.vtkTransformPolyDataFilter()
    transformFilter.SetInputConnection(sphere.GetOutputPort())
    transformFilter.SetTransform(transform)
    transformFilter.Update()

    # 2. Create a Model node seeded with the initial geometry
    modelNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'AnimatedPointCloud')
    modelNode.SetAndObservePolyData(vtk.vtkPolyData())
    modelNode.GetPolyData().DeepCopy(transformFilter.GetOutput())
    modelNode.CreateDefaultDisplayNodes()

    # 3. Set up the ROS 2 publisher
    rosLogic = slicer.util.getModuleLogic('ROS2')
    rosNode = rosLogic.GetDefaultROS2Node()
    pub = rosNode.CreateAndAddPublisherNode('PolyData', '/slicer_point_cloud')
    pub.SetFrameId('world')

    # 4. Auto-republish whenever the model node is marked modified.
    #    This also fires if the user edits the model interactively in Slicer.
    def onModelModified(caller, event):
        pub.Publish(caller.GetPolyData())

    observerTag = modelNode.AddObserver(vtk.vtkCommand.ModifiedEvent, onModelModified)

    # Publish the initial frame immediately
    pub.Publish(modelNode.GetPolyData())

    # 5. Drive the animation at 10 Hz: rotate the sphere 5 degrees per step
    angle = [0.0]

    def animate():
        angle[0] = (angle[0] + 5.0) % 360.0
        transform.Identity()
        transform.RotateZ(angle[0])
        transformFilter.Update()
        modelNode.GetPolyData().DeepCopy(transformFilter.GetOutput())
        modelNode.Modified()  # triggers onModelModified -> pub.Publish()

    timer = qt.QTimer()
    timer.setInterval(100)  # 10 Hz
    timer.connect('timeout()', animate)
    timer.start()


    print("Animated point cloud publisher started at 10 Hz on /slicer_point_cloud.")
    print("The model will also republish automatically if you edit it in Slicer.")


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
    _state.timer.stop()
    _state.timer.disconnect("timeout()", _state.animate)
    _state.timer.deleteLater()
    _state.modelNode.RemoveObserver(_state.observerTag)
    if _state.rosNode.GetScene() is not None:
        _state.rosNode.RemoveAndDeletePublisherNode("/slicer_point_cloud")
    display = _state.modelNode.GetDisplayNode()
    if _state.modelNode.GetScene() is not None:
        slicer.mrmlScene.RemoveNode(_state.modelNode)
    if display is not None and display.GetScene() is not None:
        slicer.mrmlScene.RemoveNode(display)
    _state = None


if __name__ == "__main__":
    run()
