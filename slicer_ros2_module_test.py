import slicer
import unittest
import time
import subprocess

# class TestSum(unittest.TestCase):

#     def test_sum(self):
#         self.assertEqual(sum([1, 2, 3]), 6, "Should be 6")

#     def test_sum_tuple(self):
#         self.assertEqual(sum((1, 2, 2)), 6, "Should be 6")

class TestTurtlesimNode(unittest.TestCase):

    def test_turtlesim_node_active(self):
        # Start the turtlesim node as a subprocess
        # ros2Node = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLROS2NODENode')
        # ros2Node.Create('turtlesim')
        # # Wait for a few seconds to give the node time to initialize
        # time.sleep(3)
        # # Check if the turtlesim node is running by checking the rosnode list
        # nodes = subprocess.check_output("ros2 node list", shell=True).decode("utf-8").split("\n")
        # # Assert that the turtlesim node is in the list of running nodes
        # self.assertIn("/turtlesim", nodes)

class TestCreateAndAddPubSub(unittest.TestCase):

    def test_create_and_add_pub_sub(self):
        # Start the turtlesim node as a subprocess
        print('creating a ROS2 node')
        ros2Node = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLROS2NODENode')
        print('initialize the ROS2 node')
        ros2Node.Create('testNode')
        print('add a publisher on topic test_string_xkcd')
        testPub = ros2Node.CreateAndAddPublisher('vtkMRMLROS2PublisherStringNode', 'test_string_xkcd')
        print('add a subscriber on topic test_string_xkcd')
        testSub = ros2Node.CreateAndAddSubscriber('vtkMRMLROS2SubscriberStringNode', 'test_string_xkcd')
        print('publish')
        testPub.Publish('xkcd')
        # assert true
        self.assertTrue(True)

def run():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestCreateAndAddPubSub)
    unittest.TextTestRunner().run(suite)
    print('ran 1 test')
    suite = unittest.TestLoader().loadTestsFromTestCase(TestTurtlesimNode)
    unittest.TextTestRunner().run(suite)

if __name__ == '__main__':
    slicer_ros2_module_test()
    







