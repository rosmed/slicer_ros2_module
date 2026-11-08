"""Exercise installed examples in Slicer without an external robot or MoveIt."""
import importlib
import unittest
from unittest.mock import Mock, patch

import slicer
import vtk


class ExamplesTest(unittest.TestCase):
    def test_importing_examples_does_not_change_the_scene(self):
        import slicerros2.examples
        before = slicer.mrmlScene.GetNumberOfNodes()
        for name in slicerros2.examples.__all__:
            importlib.import_module("slicerros2.examples." + name)
        self.assertEqual(slicer.mrmlScene.GetNumberOfNodes(), before)

    def test_point_cloud_examples_run_and_cleanup(self):
        from slicerros2.examples import point_cloud_publisher, point_cloud_subscriber
        with patch("slicer.util.selectModule"):
            try:
                publisher = point_cloud_publisher.run()
                self.assertTrue(publisher.timer.isActive())
                self.assertGreater(publisher.modelNode.GetPolyData().GetNumberOfPoints(), 0)
                subscriber = point_cloud_subscriber.run()
                self.assertEqual(subscriber.sub.GetTargetNodeID(), subscriber.modelNode.GetID())
                ids = [publisher.modelNode.GetID(), subscriber.modelNode.GetID()]
                publisher.animate()
                point_cloud_publisher.cleanup()
                point_cloud_subscriber.cleanup()
                self.assertTrue(all(slicer.mrmlScene.GetNodeByID(node_id) is None for node_id in ids))
            finally:
                point_cloud_publisher.cleanup()
                point_cloud_subscriber.cleanup()

    def test_shared_moveit_obstacle_geometry_and_transform(self):
        from slicerros2.examples import _moveit
        before = {node.GetID() for node in slicer.util.getNodesByClass("vtkMRMLNode")}
        logic, robot = Mock(), Mock()
        logic.AddMoveItObstacleWithTransform.return_value = (Mock(), 1)
        try:
            result = _moveit.add_obstacle(logic, robot, "world")
            self.assertEqual(result.obstacleNode.GetPolyData().GetBounds(), (-100, 100, -100, 100, -100, 100))
            self.assertEqual(result.obstacleNode.GetParentTransformNode(), result.obstacleTransform)
            pose = vtk.vtkMatrix4x4()
            result.obstacleTransform.GetMatrixTransformToParent(pose)
            self.assertEqual([pose.GetElement(i, 3) for i in range(3)], [400, 0, 300])
            logic.AddMoveItObstacleWithTransform.assert_called_once_with(
                result.obstacleNode, result.obstacleTransform, "world", robot)
        finally:
            for node in slicer.util.getNodesByClass("vtkMRMLNode"):
                if node.GetID() not in before:
                    slicer.mrmlScene.RemoveNode(node)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ExamplesTest))
    slicer.mrmlScene.Clear(0)
    vtk.vtkDebugLeaks.SetExitError(False)
    slicer.util.exit(0 if result.wasSuccessful() else 1)
