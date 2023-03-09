import slicer
import unittest
import subprocess
import logging
import sys
import psutil
import os

import vtk

# use logging to print to console
logging.basicConfig(level=logging.WARNING)

############################################## GLOBAL VARIABLES ##############################################

ENVIRONMENT_CORRECTION = "export PYTHONPATH=/opt/ros/galactic/lib/python3.8/site-packages; export PYTHONHOME=; "

ros2Module = slicer.modules.ros2
logic = ros2Module.logic()

############################################## HELPER FUNCTIONS ##############################################

def run_ros2_cli_command_blocking(command):
    ros2_process = subprocess.Popen(
        ENVIRONMENT_CORRECTION + "python3.8 /opt/ros/galactic/bin/ros2 " + command,
        shell=True,
        preexec_fn=os.setsid,
    )
    ros2_process.wait()
    return ros2_process

def run_ros2_cli_command_non_blocking(command):
    ros2_process = subprocess.Popen(
        ENVIRONMENT_CORRECTION + "python3.8 /opt/ros/galactic/bin/ros2 " + command,
        shell=True,
        preexec_fn=os.setsid,
    )
    return ros2_process

def kill_subprocess(proc):
    os.killpg(os.getpgid(proc.pid), subprocess.signal.SIGINT)

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

############################################## TESTS ##############################################

# It creates a turtlesim node, checks if it's running, and then kills it
class TestTurtlesimNode(unittest.TestCase):
    def setUp(self):
        self.create_turtlesim_node_process = run_ros2_cli_command_non_blocking("run turtlesim turtlesim_node")        

    def test_turtlesim_node_create_and_destroy(self):
        print("\nTesting creation and destruction of turtlesim node - Starting..")
        # Check if the turtlesim node is running by checking the rosnode list
        self.assertTrue(check_ros2_node_running("/turtlesim"), "Turtlesim node not running")
        print("Testing creation and destruction of turtlesim node - Done")

    def tearDown(self):
        # Kill the turtlesim node
        kill_subprocess(self.create_turtlesim_node_process)
        self.assertFalse(check_ros2_node_running("/turtlesim"), "Turtlesim node still running")
        


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

        self.assertTrue(finalSubMessageCount - initSubMessageCount == 1, "Message not received")
        # assert that the recceived message contains the string message - since YAML
        self.assertTrue(messageString in receivedMessage, "Message not received correctly")

        self.assertTrue(self.ros2Node.RemovePublisherNode("test_string_xkcd"), "Publisher not removed")
        self.assertTrue(self.ros2Node.RemoveSubscriberNode("test_string_xkcd"), "Subscriber not removed")
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
        self.assertTrue(self.ros2Node.RemovePublisherNode("test_string_xkcd"), "Publisher which exists not removed")
        spin_some()
        # delete publisher which used to exist
        self.assertFalse(self.ros2Node.RemovePublisherNode("test_string_xkcd"), "Publisher which used to exist removed")
        spin_some()
        # delete publisher which never existed
        self.assertFalse(self.ros2Node.RemovePublisherNode("random_name"), "Publisher which never existed removed")
        spin_some()
        # delete subscriber which exists
        self.assertTrue(self.ros2Node.RemoveSubscriberNode("test_string_xkcd"), "Subscriber which exists not removed")
        spin_some()
        # delete subscriber which used to exist
        self.assertFalse(self.ros2Node.RemoveSubscriberNode("test_string_xkcd"), "Subscriber which used to exist removed")
        spin_some()
        # delete subscriber which never existed
        self.assertFalse(self.ros2Node.RemoveSubscriberNode("random_name"), "Subscriber which never existed removed")
        spin_some()
        print("Testing deletion of publisher and subscriber - Done")

    def tearDown(self):
        pass
        # self.ros2Node.Destroy()

class TestParameterNode(unittest.TestCase):
    def setUp(self):
        print("\nCreating ROS2 node..")
        self.ros2Node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLROS2NodeNode")
        self.ros2Node.Create("testNode")
        self.create_turtlesim_node_process = run_ros2_cli_command_non_blocking("run turtlesim turtlesim_node")        
        spin_some()
        self.assertTrue(check_ros2_node_running("/turtlesim"), "Turtlesim node not running")
        spin_some()

    def test_parameter_monitoring(self):
        print("\nTesting creation and working of parameter node - Starting..")
        testParam = self.ros2Node.CreateAndAddParameter("/turtlesim")
        spin_some()

        # Valid Parameter which will be added
        self.assertEqual(testParam.GetParameterType("background_r"), "", "Parameter type not empty")
        self.assertEqual(testParam.GetParameterType("background_r"), "", "Parameter type not empty")

        # Parameter which does not exist
        self.assertEqual(testParam.GetParameterType("background_y"), "", "Parameter type not empty")
        self.assertEqual(testParam.GetParameterType("background_y"), "", "Parameter type not empty")

        # Parameter which exists but would not be added to monitor
        self.assertEqual(testParam.GetParameterType("background_g"), "", "Parameter type not empty")
        self.assertEqual(testParam.GetParameterType("background_g"), "", "Parameter type not empty")

        testParam.AddParameter("background_r")
        testParam.AddParameter("background_y")

        while(testParam.IsParameterSet("background_r") == False):
            spin_some()

        self.assertEqual(testParam.GetParameterType("background_r"), "integer", "Parameter type not integer")

        self.assertFalse(testParam.IsParameterSet("background_y", True), "Parameter type not empty")
        self.assertEqual(testParam.GetParameterType("background_y"), "", "Parameter type not empty")

        # Update parameter value
        change_param_value = run_ros2_cli_command_blocking(
            "param set /turtlesim background_g 150"
        )

        while(testParam.IsParameterSet("background_g", True) == False):
            spin_some()

        self.assertTrue(testParam.IsParameterSet("background_g"))
        self.assertEqual(testParam.GetParameterType("background_g"), "integer")
        self.assertEqual(testParam.GetParameterAsInteger("background_g"), 150)

        # delete parameter node
        self.assertTrue(self.ros2Node.RemoveParameterNode("/turtlesim"))
        print("Testing creation and working of parameter node - Done")

    def test_parameter_deletion(self):
        print("\nTesting deletion of parameter node - Starting..")
        testParam = self.ros2Node.CreateAndAddParameter("/turtlesim")
        spin_some()

        # delete parameter node which exists
        self.assertTrue(self.ros2Node.RemoveParameterNode("/turtlesim"), "Failed to delete parameter node which exists")
        spin_some()
        # delete parameter node which used to exist
        self.assertFalse(self.ros2Node.RemoveParameterNode("/turtlesim"), "Deleted parameter node which used to exist")
        spin_some()
        # delete parameter node which never existed
        self.assertFalse(self.ros2Node.RemoveParameterNode("/random_name"), "Deleted parameter node which never existed")
        spin_some()
        print("Testing deletion of parameter node - Done")

    def tearDown(self):
        # self.ros2Node.Destroy()
        kill_subprocess(self.create_turtlesim_node_process)

class TestBroadcasterAndLookupNode(unittest.TestCase):
    def setUp(self):
        print("\nCreating ROS2 node to test Broadcaster Nodes..")
        self.ros2Node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLROS2NodeNode")
        self.ros2Node.Create("testNode")
        broadcaster = self.ros2Node.CreateAndAddBroadcaster("vtkMRMLROS2Tf2BroadcasterNode", "Parent", "Child")
        buffer = self.ros2Node.GetTf2Buffer()
        lookupNode = buffer.CreateAndAddLookupNode("Parent", "Child")
        # Broadcast a 4x4 matrix and confirm
        broadcastedMat = vtk.vtkMatrix4x4()
        broadcastedMat.SetElement(0,3,66) # Set a default value
        lookupMat = lookupNode.GetMatrixTransformFromParent()
        self.assertTrue(lookupMat.GetElement(0,3), broadcastedMat.GetElement(0,3))



    def test_broadcaster_functioning(self):
        # Add a broadcaster node and assert its functioning is as expected
        # Do not bother with the broadcaster node's destruction - Haven't gotten to that yet
        self.assertTrue(True)
        self.assertFalse(False)
        self.assertEqual(1, 1)
        # Probably I will broadcast something to a test lookup node and make sure the transform matches
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