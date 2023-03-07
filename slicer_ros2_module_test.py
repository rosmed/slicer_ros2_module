import slicer
import unittest
import subprocess
import logging
import sys
import psutil
import os

# use logging to print to console
logging.basicConfig(level=logging.WARNING)

ENVIRONMENT_CORRECTION = "export PYTHONPATH=/opt/ros/galactic/lib/python3.8/site-packages; export PYTHONHOME=; "

ros2Module = slicer.modules.ros2
logic = ros2Module.logic()

def check_ros2_node_running(nodeName):
    # Check if the turtlesim node is running by checking the rosnode list
    nodes = (
        subprocess.check_output(
            ENVIRONMENT_CORRECTION + "python3.8 /opt/ros/galactic/bin/ros2 node list",
            shell=True,
        )
        .decode("utf-8")
        .split("\n")
    )
    # Assert that the turtlesim node is in the list of running nodes
    return nodeName in nodes


def spin_some():
    for i in range(3):
        logic.Spin()


# It creates a turtlesim node, checks if it's running, and then kills it
class TestTurtlesimNode(unittest.TestCase):
    def setUp(self):
        self.create_turtlesim_node_process = subprocess.Popen(
            ENVIRONMENT_CORRECTION
            + "python3.8 /opt/ros/galactic/bin/ros2 run turtlesim turtlesim_node",
            shell=True,
            preexec_fn=os.setsid,
        )

    def test_turtlesim_node_create_and_destroy(self):
        print("\nTesting creation and destruction of turtlesim node - Starting..")
        # Check if the turtlesim node is running by checking the rosnode list
        self.assertTrue(check_ros2_node_running("/turtlesim"))

    def tearDown(self):
        os.killpg(
            os.getpgid(self.create_turtlesim_node_process.pid), subprocess.signal.SIGINT
        )
        self.assertFalse(check_ros2_node_running("/turtlesim"))
        print("Testing creation and destruction of turtlesim node - Done")


# It creates a ROS2 node, adds a publisher and subscriber to it, and publishes a message
class TestCreateAndAddPubSub(unittest.TestCase):
    def setUp(self):
        print("\nCreating ROS2 node..")
        self.ros2Node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLROS2NodeNode")
        self.ros2Node.Create("testNode")

    def test_create_and_add_pub_sub(self):
        print("\nTesting creation and working of publisher and subscriber - Starting..")
        testPub = self.ros2Node.CreateAndAddPublisher(
            "vtkMRMLROS2PublisherStringNode", "test_string_xkcd"
        )
        testSub = self.ros2Node.CreateAndAddSubscriber(
            "vtkMRMLROS2SubscriberStringNode", "test_string_xkcd"
        )
        spin_some()

        initSubMessageCount = testSub.GetNumberOfMessages()
        messageString = "xkcd"
        testPub.Publish(messageString)
        spin_some()

        finalSubMessageCount = testSub.GetNumberOfMessages()
        receivedMessage = testSub.GetLastMessageYAML() # TODO: GetLastMessageString()

        self.assertTrue(finalSubMessageCount - initSubMessageCount == 1)
        # assert that the recceived message contains the string message - since YAML
        self.assertTrue(messageString in receivedMessage)

        self.assertTrue(self.ros2Node.RemovePublisherNode("test_string_xkcd"))
        self.assertTrue(self.ros2Node.RemoveSubscriberNode("test_string_xkcd"))
        print("Testing creation and working of publisher and subscriber - Done")

    def test_pub_sub_deletion(self):
        print("\nTesting deletion of publisher and subscriber - Starting..")
        testPub = self.ros2Node.CreateAndAddPublisher(
            "vtkMRMLROS2PublisherStringNode", "test_string_xkcd"
        )
        testSub = self.ros2Node.CreateAndAddSubscriber(
            "vtkMRMLROS2SubscriberStringNode", "test_string_xkcd"
        )

        # delete publisher which exists
        self.assertTrue(self.ros2Node.RemovePublisherNode("test_string_xkcd"))
        spin_some()
        # delete publisher which used to exist
        self.assertFalse(self.ros2Node.RemovePublisherNode("test_string_xkcd"))
        spin_some()
        # delete publisher which never existed
        self.assertFalse(self.ros2Node.RemovePublisherNode("random_name"))
        spin_some()
        # delete subscriber which exists
        self.assertTrue(self.ros2Node.RemoveSubscriberNode("test_string_xkcd"))
        spin_some()
        # delete subscriber which used to exist
        self.assertFalse(self.ros2Node.RemoveSubscriberNode("test_string_xkcd"))
        spin_some()
        # delete subscriber which never existed
        self.assertFalse(self.ros2Node.RemoveSubscriberNode("random_name"))
        spin_some()
        print("Testing deletion of publisher and subscriber - Done")

    def tearDown(self):
        pass
        # self.ros2Node.Destroy()

class TestBroadcasterNode(unittest.TestCase):
    def setUp(self):
        print("\nCreating ROS2 node to test Broadcaster Nodes..")
        self.ros2Node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLROS2NodeNode")
        self.ros2Node.Create("testNode")

    def test_broadcaster_functioning(self):
        # Add a broadcaster node and assert its functioning is as expected
        # Do not bother with the broadcaster node's destruction - Haven't gotten to that yet
        self.assertTrue(True)
        self.assertFalse(False)
        self.assertEqual(1, 1)
        pass

    def tearDown(self):
        pass
        # self.ros2Node.Destroy()

class TestBufferNode(unittest.TestCase):
    def setUp(self):
        print("\nCreating ROS2 node to test Buffer Nodes..")
        self.ros2Node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLROS2NodeNode")
        self.ros2Node.Create("testNode")

    def test_buffer_functioning(self):
        # Add a buffer node and assert its functioning is as expected
        # Do not bother with the buffer node's destruction - Haven't gotten to that yet
        self.assertTrue(True)
        self.assertFalse(False)
        self.assertEqual(1, 1)
        pass

    def tearDown(self):
        pass
        # self.ros2Node.Destroy()


def run():
    suite = unittest.TestLoader()
    # load all tests in this file
    suite = suite.loadTestsFromModule(sys.modules[__name__])
    unittest.TextTestRunner().run(suite)


if __name__ == "__main__":
    pass