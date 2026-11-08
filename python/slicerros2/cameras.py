"""Render the Slicer scene from a tracked camera into an RGB MRML volume."""

import logging
import math
import uuid


class VirtualCamera:
    """A dedicated offscreen Slicer view with a live RGB image output.

    The supplied MRML transform maps an optical frame into Slicer world
    coordinates (millimeters): +X right, +Y down, +Z forward. Its full transform
    hierarchy is used, including roll. The main Slicer views are unchanged.

    ``image_node`` is a 2D vtkMRMLVectorVolumeNode, dimensions (width, height, 1),
    with three unsigned-byte RGB components. Its pixels are not a spatially
    calibrated medical volume. Image I/J point right/up in a standard axial
    slice view. Rendering and all methods run on the Qt thread.
    Keep the returned object and call close() to release its view and timer.
    """

    def __init__(self, transform_node, *, name="Virtual camera", width=640,
                 height=480, fps=20.0, horizontal_fov_degrees=60.0,
                 near_mm=5.0, far_mm=10000.0, start=True):
        if int(width) != width or int(height) != height or width <= 0 or height <= 0:
            raise ValueError("Camera width and height must be positive integers.")
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("Camera fps must be finite and positive.")
        if not 0 < horizontal_fov_degrees < 180:
            raise ValueError("Horizontal FOV must be between 0 and 180 degrees.")
        if not math.isfinite(near_mm) or not math.isfinite(far_mm) or not 0 < near_mm < far_mm:
            raise ValueError("Clipping planes must satisfy 0 < near_mm < far_mm.")

        import qt
        import slicer
        import vtk

        if (transform_node is None or not transform_node.IsA("vtkMRMLTransformNode")
                or transform_node.GetScene() != slicer.mrmlScene):
            raise ValueError("The optical transform must belong to the current scene.")
        if not transform_node.GetMatrixTransformToWorld(vtk.vtkMatrix4x4()):
            raise ValueError("The optical transform must have a linear transform hierarchy.")
        self.transform_node = transform_node
        self.width, self.height = int(width), int(height)
        self.near_mm, self.far_mm = near_mm, far_mm
        self._closed = False
        self._on_close_callback = None
        self.last_error = None
        self.frame_count = 0
        self._scene = slicer.mrmlScene
        self.image_node = self._scene.AddNewNodeByClass("vtkMRMLVectorVolumeNode", name + " RGB")
        # Standard axial slice views use -R for screen-right and +A for up.
        self.image_node.SetIJKToRASDirections(-1., 0., 0., 0., 1., 0., 0., 0., 1.)
        self.image_node.CreateDefaultDisplayNodes()
        self._image_display = self.image_node.GetDisplayNode()
        self.image_node.SetAttribute("slicerros2.camera.horizontal_fov_degrees", str(horizontal_fov_degrees))
        self.image_node.SetAttribute("slicerros2.camera.optical_axes", "+X right, +Y down, +Z forward")
        self.view_node = slicer.vtkMRMLViewNode()
        self.view_node.SetName(name + " view")
        # LayoutName is the singleton tag and must be set before AddNode assigns an ID.
        self.view_node.SetLayoutName("slicerros2_" + uuid.uuid4().hex)
        self.view_node.SetHideFromEditors(True)
        self.view_node.SetBoxVisible(False)
        self.view_node.SetAxisLabelsVisible(False)
        self.view_node.SetBackgroundColor(0, 0, 0)
        self.view_node.SetBackgroundColor2(0, 0, 0)
        self._scene.AddNode(self.view_node)
        self._widget = slicer.qMRMLThreeDWidget()
        self._widget.setAttribute(qt.Qt.WA_DontShowOnScreen, True)
        self._widget.setMRMLScene(self._scene)
        self._widget.setMRMLViewNode(self.view_node)
        self._widget.resize(self.width, self.height)
        self._widget.show()
        self._window = self._widget.threeDView().renderWindow()
        self._window.SetOffScreenRendering(1)
        self._window.SetSize(self.width, self.height)
        self.camera_node = slicer.modules.cameras.logic().GetViewActiveCameraNode(self.view_node)
        self.camera_node.SetHideFromEditors(True)
        self.camera_node.GetCamera().UseHorizontalViewAngleOn()
        self.camera_node.GetCamera().SetViewAngle(horizontal_fov_degrees)
        self._capture = vtk.vtkWindowToImageFilter()
        self._capture.SetInput(self._window)
        self._capture.SetInputBufferTypeToRGB()
        self._capture.ReadFrontBufferOff()
        self._capture.ShouldRerenderOff()
        self._pixels = vtk.vtkImageData()
        self._timer = qt.QTimer(slicer.app)
        self._timer.setInterval(max(1, round(1000.0 / fps)))
        self._timer.connect("timeout()", self._tick)
        self._scene_observer = self._scene.AddObserver(self._scene.StartCloseEvent, self._on_scene_close)
        if start:
            try:
                self.start()
            except Exception:
                self.close(remove_image=True)
                raise

    @property
    def running(self):
        """Whether automatic image capture is active."""
        return not self._closed and self._timer.isActive()

    def capture(self):
        """Update the optical pose and return one RGB frame in image_node."""
        import vtk

        if self._closed:
            raise RuntimeError("This camera has been closed.")
        if self.transform_node.GetScene() != self._scene or self.image_node.GetScene() != self._scene:
            raise RuntimeError("The camera transform or image has been removed from the scene.")
        pose = vtk.vtkMatrix4x4()
        if not self.transform_node.GetMatrixTransformToWorld(pose):
            raise RuntimeError("The optical transform must have a linear transform hierarchy.")
        position = [pose.GetElement(i, 3) for i in range(3)]
        forward = [pose.GetElement(i, 2) for i in range(3)]
        up = [-pose.GetElement(i, 1) for i in range(3)]
        camera = self.camera_node.GetCamera()
        camera.SetPosition(*position)
        camera.SetFocalPoint(*[position[i] + forward[i] for i in range(3)])
        camera.SetViewUp(*up)
        camera.SetClippingRange(self.near_mm, self.far_mm)
        self._window.SetSize(self.width, self.height)
        self._window.Render()
        self._capture.Modified()
        self._capture.Update()
        self._pixels.DeepCopy(self._capture.GetOutput())
        self.image_node.SetAndObserveImageData(self._pixels)
        self.image_node.Modified()
        self.frame_count += 1
        return self.image_node

    def start(self):
        """Capture immediately and start periodic updates."""
        self.capture()
        self.last_error = None
        self._timer.start()
        return self

    def stop(self):
        """Pause updates, retaining the last image and camera resources."""
        if not self._closed:
            self._timer.stop()

    def _tick(self):
        try:
            self.capture()
        except Exception as error:
            self.last_error = error
            logging.exception("Virtual camera stopped")
            self.close()

    def _on_scene_close(self, caller, event):
        self.close()

    def close(self, *, remove_image=False):
        """Stop updates and release the view; optionally remove the RGB image.

        The existing robot transform is never removed. The last frame remains
        usable when remove_image is False. A later call can still remove the image.
        """
        if not self._closed:
            self.stop()
            self._closed = True
            self._timer.disconnect("timeout()", self._tick)
            self._timer.deleteLater()
            self._scene.RemoveObserver(self._scene_observer)
            self._capture.SetInput(None)
            self._widget.close()
            self._widget.setMRMLScene(None)
            self._window.Finalize()
            self._widget.deleteLater()
            for node in (self.camera_node, self.view_node):
                if node.GetScene() == self._scene:
                    self._scene.RemoveNode(node)
            if self._on_close_callback is not None:
                callback, self._on_close_callback = self._on_close_callback, None
                callback()
        if remove_image:
            for node in (self.image_node, self._image_display):
                if node.GetScene() == self._scene:
                    self._scene.RemoveNode(node)
