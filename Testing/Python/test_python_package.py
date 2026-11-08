"""Package import/documentation checks; run with system Python."""
import ast
import importlib
from pathlib import Path
import shlex
import sys
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))


class PythonPackageTests(unittest.TestCase):
    def test_all_documented_examples_import_without_slicer_side_effects(self):
        import slicerros2.examples
        names = {"point_cloud_publisher", "point_cloud_subscriber", "moveit_obstacle",
                 "moveit_custom_tool", "moveit_pose_markers", "patient_cart", "ecm_camera"}
        self.assertEqual(set(slicerros2.examples.__all__), names)
        with patch.dict(sys.modules, {"slicer": object(), "qt": object(), "vtk": object()}):
            for name in names:
                module = importlib.import_module("slicerros2.examples." + name)
                self.assertTrue(callable(module.run), name)
            from slicerros2.dvrk import PatientCart, ECMCamera
            self.assertTrue(isinstance(PatientCart, type))
            self.assertTrue(isinstance(ECMCamera, type))

    def test_patient_cart_load_reuses_nodes_and_unload_releases_them(self):
        from slicerros2.dvrk import PatientCart
        node = Mock()
        node.GetRobotNodeByName.return_value = None
        with patch("slicerros2.dvrk.get_ros_node", return_value=node):
            cart = PatientCart()
            robots = cart.load()
            self.assertEqual(set(robots), {"PSM1", "PSM2", "PSM3", "ECM"})
            self.assertEqual(node.CreateAndAddRobotNode.call_count, 4)
            node.GetRobotNodeByName.side_effect = lambda name: robots.get(name)
            cart.load()
            self.assertEqual(node.CreateAndAddRobotNode.call_count, 4)
            cart.unload()
            self.assertEqual(node.RemoveAndDeleteRobotNode.call_count, 4)
            self.assertFalse(cart.robots)

    def test_patient_cart_load_options_are_independent(self):
        from slicerros2.dvrk import PatientCart, load_patient_cart
        for include_ECM in (False, True):
            for include_SUJ in (False, True):
                with self.subTest(include_ECM=include_ECM, include_SUJ=include_SUJ):
                    node = Mock()
                    node.GetRobotNodeByName.return_value = None
                    expected = {"PSM1", "PSM2", "PSM3"}
                    if include_ECM:
                        expected.add("ECM")
                    if include_SUJ:
                        expected.add("SUJ")
                    cart = PatientCart(ros_node=node, include_ECM=include_ECM, include_SUJ=include_SUJ)
                    robots = cart.load()
                    self.assertEqual(set(robots), expected)
                    self.assertEqual({call.args[0] for call in node.CreateAndAddRobotNode.call_args_list}, expected)
                    node.GetRobotNodeByName.return_value = Mock()
                    cart.unload()
                    self.assertEqual({call.args[0] for call in node.RemoveAndDeleteRobotNode.call_args_list}, expected)
                    with patch("slicerros2.dvrk.get_ros_node", return_value=node):
                        self.assertEqual(set(load_patient_cart(include_ECM=include_ECM, include_SUJ=include_SUJ)), expected)

    def test_timed_out_patient_cart_can_be_unloaded(self):
        from slicerros2.dvrk import PatientCart

        robots = {}
        node = Mock()
        node.GetRobotNodeByName.side_effect = robots.get

        def create_robot(name, *args):
            robots[name] = Mock()
            return robots[name]

        node.CreateAndAddRobotNode.side_effect = create_robot
        node.RemoveAndDeleteRobotNode.side_effect = robots.pop
        cart = PatientCart(ros_node=node)
        with patch("slicerros2.dvrk.wait_for_models", side_effect=TimeoutError("no geometry")):
            with self.assertRaises(TimeoutError):
                cart.load(wait=True, timeout=0)
        self.assertEqual(set(cart.robots), {"PSM1", "PSM2", "PSM3", "ECM"})
        cart.unload()
        self.assertFalse(robots)
        self.assertFalse(cart.robots)

    def test_camera_matches_full_frames_before_unqualified_suffixes(self):
        from slicerros2.dvrk import _camera_transform

        def lookup(frame):
            result = Mock()
            result.GetChildID.return_value = frame
            return result

        prefixed = lookup("cart/ECM_optical")
        exact = lookup("ECM_optical")
        robot = Mock()
        robot.GetNumberOfNodeReferences.return_value = 2
        robot.GetNthNodeReference.side_effect = lambda role, index: [prefixed, exact][index]
        for frame, expected in (("cart/ECM_optical", prefixed), ("ECM_optical", exact)):
            with self.subTest(frame=frame):
                self.assertEqual(_camera_transform(robot, tip_frame=frame), (expected, None))

        robot.GetNumberOfNodeReferences.return_value = 1
        self.assertEqual(_camera_transform(robot), (prefixed, None))
        with self.assertRaises(RuntimeError):
            _camera_transform(robot, tip_frame="other/ECM_optical")

    def test_camera_example_can_omit_ecm_visuals(self):
        from slicerros2.examples import ecm_camera
        with patch("slicerros2.dvrk.load_patient_cart", return_value={}) as load, \
                patch("slicerros2.dvrk.ECMCamera") as camera:
            try:
                result = ecm_camera.run(include_ECM=False, include_SUJ=True)
                self.assertIs(result, camera.return_value)
                load.assert_called_once_with(include_SUJ=True, include_ECM=False, fixed_frame="world", wait=True)
                camera.assert_called_once_with(None, fixed_frame="world")
            finally:
                ecm_camera.cleanup()

    def test_documentation_includes_canonical_package_sources(self):
        sources = set()
        for page in (ROOT / "docs/pages/examples").glob("*.rst"):
            for line in page.read_text().splitlines():
                if line.startswith(".. literalinclude::"):
                    source = (page.parent / line.split("::", 1)[1].strip()).resolve()
                    self.assertTrue(source.is_file(), (page, source))
                    self.assertTrue(source.is_relative_to(ROOT / "python/slicerros2"))
                    sources.add(source.stem)
        self.assertTrue({"point_cloud_publisher", "point_cloud_subscriber", "_moveit",
                         "moveit_custom_tool", "moveit_pose_markers"}.issubset(sources))
        self.assertFalse((ROOT / "docs/code").exists() and list((ROOT / "docs/code").glob("*.py")))

    def test_dvrk_walkthrough_and_helper_reference_are_linked(self):
        examples = (ROOT / "docs/pages/examples.rst").read_text()
        walkthrough = (ROOT / "docs/pages/examples/dvrk.rst").read_text()
        helpers = (ROOT / "docs/pages/utilities/dvrk.rst").read_text()
        self.assertIn("   examples/dvrk", examples)
        self.assertIn(":doc:`/pages/utilities/dvrk`", walkthrough)
        self.assertIn(":doc:`/pages/examples/dvrk`", helpers)

    def test_startup_stubs_only_import_and_run_the_packaged_examples(self):
        import slicerros2.examples
        for name in slicerros2.examples.__all__:
            tree = ast.parse((ROOT / "examples" / (name + ".py")).read_text())
            self.assertEqual(len(tree.body), 3)  # docstring, import, run
            self.assertEqual(tree.body[1].module, "slicerros2.examples")
            self.assertEqual(tree.body[1].names[0].name, name)
            self.assertEqual(tree.body[2].value.func.attr, "run")

    def test_documented_stub_command_keeps_the_script_path_together(self):
        lines = (ROOT / "docs/pages/examples.rst").read_text().splitlines()
        line = next(line.strip() for line in lines if line.strip().startswith("ros2 run") and "--python-script" in line)
        args = shlex.split(line)
        self.assertEqual(args[:4], ["ros2", "run", "slicer_ros2_module", "slicer"])
        self.assertEqual(args[4], "--python-script")
        self.assertTrue(args[5].endswith("/examples/point_cloud_publisher.py"))
        self.assertEqual(len(args), 6)


if __name__ == "__main__":
    unittest.main()
