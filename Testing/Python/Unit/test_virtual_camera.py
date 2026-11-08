"""Run in Slicer with --python-script; no ROS graph or dVRK is required."""

import time
import unittest
from unittest.mock import Mock, patch

import numpy as np
import slicer
import vtk

from slicerros2.cameras import VirtualCamera


class VirtualCameraTest(unittest.TestCase):
    def setUp(self):
        self.nodes = []
        self.camera = None
        self.parent = self.add_node("vtkMRMLLinearTransformNode")
        self.optical = self.add_node("vtkMRMLLinearTransformNode")
        self.optical.SetAndObserveTransformNodeID(self.parent.GetID())
        # ROS optical axes expressed in the original +X-facing test scene.
        self.optical_axes = vtk.vtkMatrix4x4()
        self.optical_axes.DeepCopy((0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1))
        self.optical.SetMatrixTransformToParent(self.optical_axes)
        pose = vtk.vtkMatrix4x4()
        for i, value in enumerate((10, 20, 30)):
            pose.SetElement(i, 3, value)
        self.parent.SetMatrixTransformToParent(pose)
        for center, color in (((210, 20, 30), (1, 0, 0)),
                              ((10, 220, 30), (0, 0, 1))):
            sphere = vtk.vtkSphereSource()
            sphere.SetCenter(*center)
            sphere.SetRadius(20)
            sphere.Update()
            model = self.add_node("vtkMRMLModelNode")
            model.SetAndObservePolyData(sphere.GetOutput())
            model.CreateDefaultDisplayNodes()
            display = model.GetDisplayNode()
            self.nodes.append(display)
            display.SetColor(*color)
            display.SetAmbient(1)
            display.SetDiffuse(0)

    def add_node(self, class_name):
        node = slicer.mrmlScene.AddNewNodeByClass(class_name)
        self.nodes.append(node)
        return node

    def tearDown(self):
        if self.camera is not None:
            self.camera.close(remove_image=True)
        for node in reversed(self.nodes):
            if node.GetScene() is not None:
                slicer.mrmlScene.RemoveNode(node)

    def make_camera(self):
        self.camera = VirtualCamera(self.optical, width=160, height=120, fps=20, start=False)
        return self.camera

    def test_rgb_and_parent_transform(self):
        camera = self.make_camera()
        image = camera.capture()
        self.assertTrue(image.IsA("vtkMRMLVectorVolumeNode"))
        self.assertEqual(image.GetImageData().GetDimensions(), (160, 120, 1))
        pixels = slicer.util.arrayFromVolume(image)
        self.assertEqual(pixels.shape, (1, 120, 160, 3))
        self.assertEqual(pixels.dtype, np.uint8)
        self.assertGreater(pixels[0, 60, 80, 0], 200)
        self.assertLess(pixels[0, 60, 80, 2], 20)

        # Move the parent frame: forward becomes +Y and the blue target is seen.
        pose = vtk.vtkMatrix4x4()
        pose.SetElement(0, 0, 0)
        pose.SetElement(1, 0, 1)
        pose.SetElement(0, 1, -1)
        pose.SetElement(1, 1, 0)
        for i, value in enumerate((10, 20, 30)):
            pose.SetElement(i, 3, value)
        self.parent.SetMatrixTransformToParent(pose)
        camera.capture()
        pixels = slicer.util.arrayFromVolume(image)
        self.assertGreater(pixels[0, 60, 80, 2], 200)
        self.assertLess(pixels[0, 60, 80, 0], 20)
        np.testing.assert_allclose(camera.camera_node.GetCamera().GetPosition(), (10, 20, 30))
        np.testing.assert_allclose(camera.camera_node.GetCamera().GetDirectionOfProjection(), (0, 1, 0))

        # ECM roll must be retained instead of reconstructing a fixed world-up.
        roll = vtk.vtkMatrix4x4()
        roll.SetElement(1, 1, 0)
        roll.SetElement(2, 1, 1)
        roll.SetElement(1, 2, -1)
        roll.SetElement(2, 2, 0)
        vtk.vtkMatrix4x4.Multiply4x4(roll, self.optical_axes, roll)
        self.optical.SetMatrixTransformToParent(roll)
        camera.capture()
        np.testing.assert_allclose(camera.camera_node.GetCamera().GetViewUp(), (1, 0, 0), atol=1e-8)

    def test_image_right_and_up_match_axial_display(self):
        # Markers above and right of the optical axis distinguish rotation and reflection.
        for center, color in (((210, 20, 80), (0, 1, 0)), ((210, -30, 30), (1, 1, 0))):
            sphere = vtk.vtkSphereSource()
            sphere.SetCenter(*center)
            sphere.SetRadius(10)
            sphere.Update()
            model = self.add_node("vtkMRMLModelNode")
            model.SetAndObservePolyData(sphere.GetOutput())
            model.CreateDefaultDisplayNodes()
            display = model.GetDisplayNode()
            self.nodes.append(display)
            display.SetColor(*color)
            display.SetAmbient(1)
            display.SetDiffuse(0)
        camera = self.make_camera()
        camera.capture()
        pixels = slicer.util.arrayFromVolume(camera.image_node)[0]
        green = np.argwhere((pixels[:, :, 1] > 200) & (pixels[:, :, 0] < 20))
        yellow = np.argwhere((pixels[:, :, 0] > 200) & (pixels[:, :, 1] > 200))
        self.assertGreater(len(green), 0)
        self.assertGreater(len(yellow), 0)
        self.assertGreater(green[:, 0].mean(), camera.height / 2)
        self.assertGreater(yellow[:, 1].mean(), camera.width / 2)
        slice_node = self.add_node("vtkMRMLSliceNode")
        slice_node.SetOrientationToAxial()
        image_axes = vtk.vtkMatrix4x4()
        camera.image_node.GetIJKToRASMatrix(image_axes)
        for row in range(3):
            for col in range(2):
                self.assertAlmostEqual(image_axes.GetElement(row, col),
                                       slice_node.GetSliceToRAS().GetElement(row, col))

    def test_timer_and_cleanup(self):
        camera = self.make_camera()
        camera.start()
        initial = camera.frame_count
        deadline = time.monotonic() + 2
        while camera.frame_count <= initial and time.monotonic() < deadline:
            slicer.app.processEvents()
            time.sleep(0.01)
        self.assertGreater(camera.frame_count, initial)
        camera.stop()
        self.assertFalse(camera.running)
        view_id = camera.view_node.GetID()
        camera_id = camera.camera_node.GetID()
        image_id = camera.image_node.GetID()
        camera.close(remove_image=True)
        camera.close()
        self.assertFalse(camera.running)
        for node_id in (view_id, camera_id, image_id):
            self.assertIsNone(slicer.mrmlScene.GetNodeByID(node_id))
        self.assertIsNotNone(self.optical.GetScene())

    def test_invalid_options_create_no_nodes(self):
        count = slicer.mrmlScene.GetNumberOfNodes()
        for options in (dict(width=0), dict(fps=0), dict(horizontal_fov_degrees=180),
                        dict(near_mm=10, far_mm=5)):
            with self.subTest(options=options), self.assertRaises(ValueError):
                VirtualCamera(self.optical, **options)
        self.assertEqual(slicer.mrmlScene.GetNumberOfNodes(), count)

    def test_image_can_be_removed_after_camera_is_closed(self):
        camera = self.make_camera()
        camera.capture()
        image_id = camera.image_node.GetID()
        display_id = camera.image_node.GetDisplayNodeID()
        camera.close()
        self.assertIsNotNone(slicer.mrmlScene.GetNodeByID(image_id))
        camera.close(remove_image=True)
        camera.close(remove_image=True)
        for node_id in (image_id, display_id):
            self.assertIsNone(slicer.mrmlScene.GetNodeByID(node_id))

    def test_image_can_be_removed_after_capture_failure(self):
        camera = self.make_camera()
        image_id = camera.image_node.GetID()
        display_id = camera.image_node.GetDisplayNodeID()
        slicer.mrmlScene.RemoveNode(self.optical)
        with self.assertLogs(level="ERROR"):
            camera._tick()
        self.assertIsInstance(camera.last_error, RuntimeError)
        self.assertFalse(camera.running)
        self.assertIsNotNone(slicer.mrmlScene.GetNodeByID(image_id))
        camera.close(remove_image=True)
        for node_id in (image_id, display_id):
            self.assertIsNone(slicer.mrmlScene.GetNodeByID(node_id))

    def test_failed_start_releases_nodes(self):
        count = slicer.mrmlScene.GetNumberOfNodes()
        with patch.object(VirtualCamera, "start", side_effect=RuntimeError("capture failed")):
            with self.assertRaisesRegex(RuntimeError, "capture failed"):
                VirtualCamera(self.optical)
        self.assertEqual(slicer.mrmlScene.GetNumberOfNodes(), count)

    def test_ecm_prefers_explicit_optical_frame(self):
        from slicerros2.dvrk import create_ecm_camera

        tip, optical = Mock(), Mock()
        tip.GetChildID.return_value = "ECM_tip_link"
        optical.GetChildID.return_value = "ECM_optical"
        robot = Mock()
        robot.GetNumberOfNodeReferences.return_value = 2
        robot.GetNthNodeReference.side_effect = [tip, optical]
        with patch("slicerros2.cameras.VirtualCamera.__init__", return_value=None) as constructor:
            create_ecm_camera(robot, start=False)
        constructor.assert_called_once_with(optical, name="ECM camera", start=False)

    def test_camera_without_robot_owns_and_cleans_up_its_tf_lookup(self):
        from slicerros2.dvrk import create_ecm_camera
        node = Mock()
        node.GetRobotNodeByName.return_value = None
        node.CreateAndAddTf2LookupNode.return_value = self.optical
        lookup_id = self.optical.GetID()
        with patch("slicerros2.dvrk.get_ros_node", return_value=node):
            self.camera = create_ecm_camera(start=False)
        node.CreateAndAddTf2LookupNode.assert_called_once_with("world", "ECM_optical")
        self.camera.capture()
        self.camera.close()
        node.RemoveAndDeleteTf2LookupNode.assert_called_once_with(lookup_id)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(VirtualCameraTest))
    print("VIRTUAL_CAMERA_TESTS_PASSED" if result.wasSuccessful() else "VIRTUAL_CAMERA_TESTS_FAILED", flush=True)
    slicer.mrmlScene.Clear(0)
    vtk.vtkDebugLeaks.SetExitError(False)
    slicer.util.exit(0 if result.wasSuccessful() else 1)
