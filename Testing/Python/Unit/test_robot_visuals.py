"""Run in native-ROS Slicer with --python-script; starts its own parameter server."""

from pathlib import Path
import sys


def serve_description():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=[])
    node = Node("slicer_robot_visual_fixture")
    node.declare_parameter("robot_description", Path(sys.argv[-1]).read_text())
    Path(sys.argv[-1] + ".ready").touch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if "--serve" in sys.argv:
    serve_description()
    sys.exit(0)

import subprocess
import tempfile
import time
import unittest

import numpy as np
import slicer
import vtk


def wait_for(predicate):
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        slicer.app.processEvents()
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError("Timed out waiting for the robot visual fixture")


class RobotVisualTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assert hasattr(slicer.modules, "ros2"), "Load the ROS2 module before running these tests"
        cls.directory = tempfile.TemporaryDirectory(prefix="slicer-robot-visuals-")
        mesh = Path(cls.directory.name) / "box.stl"
        cube = vtk.vtkCubeSource()
        cube.SetXLength(.01)
        cube.SetYLength(.02)
        cube.SetZLength(.03)
        cube.Update()
        writer = vtk.vtkSTLWriter()
        writer.SetFileName(str(mesh))
        writer.SetInputConnection(cube.GetOutputPort())
        writer.Write()
        description = Path(cls.directory.name) / "robot.urdf"
        description.write_text(f'''<robot name="visual_test">
          <material name="shaft"><color rgba=".2 .3 .4 .75"/></material>
          <link name="base">
            <visual><origin xyz="0 0 -.2"/>
              <geometry><cylinder radius=".004" length=".4"/></geometry><material name="shaft"/></visual>
            <visual><origin xyz=".01 .02 .03" rpy="0 0 1.5707963267948966"/>
              <geometry><box size=".01 .02 .03"/></geometry>
              <material name="box"><color rgba="1 0 0 1"/></material></visual>
            <visual><origin xyz="-.1 0 .05"/>
              <geometry><sphere radius=".015"/></geometry></visual>
            <visual><origin xyz=".1 -.2 .3"/>
              <geometry><mesh filename="{mesh}" scale="2 3 4"/></geometry></visual>
          </link>
          <joint name="joint" type="revolute"><parent link="base"/><child link="tip"/>
            <axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
          <link name="tip"><visual><origin xyz=".02 .03 .04" rpy="0 1.5707963267948966 0"/>
            <geometry><cylinder radius=".002" length=".1"/></geometry></visual></link>
        </robot>''')
        # Restore the host environment instead of filtering machine-specific
        # Slicer paths out of its bundled Python/library environment.
        env = slicer.util.startupEnvironment()
        env.pop("PYTHONHOME", None)
        cls.server = subprocess.Popen(["/usr/bin/python3", __file__, "--serve", str(description)], env=env)
        cls.addClassCleanup(cls.directory.cleanup)
        cls.addClassCleanup(cls.stop_server)
        wait_for(lambda: Path(str(description) + ".ready").exists())
        cls.node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLROS2NodeNode")
        cls.node.Create("slicer_robot_visual_test")
        cls.addClassCleanup(cls.remove_node)
        cls.robot = cls.node.CreateAndAddRobotNode(
            "visual_test", "/slicer_robot_visual_fixture", "robot_description", "", "")
        wait_for(lambda: cls.robot.GetNumberOfNodeReferences("model") > 0)

    @classmethod
    def stop_server(cls):
        cls.server.terminate()
        cls.server.wait(timeout=10)

    @classmethod
    def remove_node(cls):
        cls.node.RemoveAndDeleteRobotNode("visual_test")
        cls.node.Destroy()
        slicer.mrmlScene.RemoveNode(cls.node)
        cls.robot = None
        cls.node = None

    def models(self):
        return {self.robot.GetNthNodeReference("model", i).GetName():
                self.robot.GetNthNodeReference("model", i)
                for i in range(self.robot.GetNumberOfNodeReferences("model"))}

    def test_geometry_bounds_materials_and_link_transforms(self):
        models = self.models()
        expected = {
            "base_model_0": [-4, 4, -4, 4, -400, 0],
            "base_model_1": [0, 20, 15, 25, 15, 45],
            "base_model_2": [-115, -85, -15, 15, 35, 65],
            "base_model_3": [90, 110, -230, -170, 240, 360],
            "tip_model_0": [-30, 70, 28, 32, 38, 42],
        }
        self.assertEqual(set(models), set(expected))
        for name, bounds in expected.items():
            np.testing.assert_allclose(models[name].GetPolyData().GetBounds(), bounds, atol=1e-4, err_msg=name)
            lookup = models[name].GetParentTransformNode()
            self.assertIsNotNone(lookup)
            self.assertEqual(lookup.GetChildID(), name.split("_model_")[0])
        shaft_display = models["base_model_0"].GetDisplayNode()
        np.testing.assert_allclose(shaft_display.GetColor(), [.2, .3, .4])
        self.assertAlmostEqual(shaft_display.GetOpacity(), .75)
        np.testing.assert_allclose(models["base_model_1"].GetDisplayNode().GetColor(), [1, 0, 0])
        self.assertEqual(models["tip_model_0"].GetParentTransformNode().GetParentTransformNode(),
                         models["base_model_0"].GetParentTransformNode())

    def test_rebuild_removes_all_visual_parts(self):
        old_ids = {model.GetID() for model in self.models().values()}
        self.robot.SetupRobotVisualization()
        self.assertEqual(len(self.models()), 5)
        self.assertTrue(all(slicer.mrmlScene.GetNodeByID(node_id) is None for node_id in old_ids))
        self.test_geometry_bounds_materials_and_link_transforms()


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(RobotVisualTest))
    slicer.mrmlScene.Clear(0)
    vtk.vtkDebugLeaks.SetExitError(False)
    slicer.util.exit(0 if result.wasSuccessful() else 1)
