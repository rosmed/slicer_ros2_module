import slicer
import unittest
import time
import subprocess
import logging
import sys
import psutil
# use logging to print to console
logging.basicConfig(level=logging.WARNING)

ENVIRONMENT_CORRECTION = "export PYTHONPATH=/opt/ros/galactic/lib/python3.8/site-packages; export PYTHONHOME=; "

## Helper Functions
def check_ros2_node_running(nodeName):
    # Check if the turtlesim node is running by checking the rosnode list
    nodes = subprocess.check_output(ENVIRONMENT_CORRECTION + "python3.8 /opt/ros/galactic/bin/ros2 node list", shell = True).decode('utf-8').split('\n')
    # Assert that the turtlesim node is in the list of running nodes
    return nodeName in nodes

def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


# It creates a turtlesim node, checks if it's running, and then kills it
class TestTurtlesimNode(unittest.TestCase):

    def setUp(self):
        self.create_turtlesim_node_process = subprocess.Popen(ENVIRONMENT_CORRECTION + "python3.8 /opt/ros/galactic/bin/ros2 run turtlesim turtlesim_node", shell = True)
        time.sleep(1)

    def test_turtlesim_node_create_and_destroy(self):
        # Check if the turtlesim node is running by checking the rosnode list
        self.assertTrue(check_ros2_node_running("/turtlesim"))

    def tearDown(self):
        # Kill the turtlesim node
        kill(self.create_turtlesim_node_process.pid)
        time.sleep(1)

# It creates a ROS2 node, adds a publisher and subscriber to it, and publishes a message
class TestCreateAndAddPubSub(unittest.TestCase):

    def setUp(self):
        print('creating a ROS2 node')
        self.ros2Node = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLROS2NodeNode')
        print('initialize the ROS2 node')
        self.ros2Node.Create('testNode')
        print('add a publisher on topic test_string_xkcd')
    
    def test_create_and_add_pub_sub(self):
        testPub = self.ros2Node.CreateAndAddPublisher('vtkMRMLROS2PublisherStringNode', 'test_string_xkcd')
        print('add a subscriber on topic test_string_xkcd')
        testSub = self.ros2Node.CreateAndAddSubscriber('vtkMRMLROS2SubscriberStringNode', 'test_string_xkcd')

        initSubMessageCount = testSub.GetNumberOfMessages()

        print('publish')
        messageString = 'xkcd'
        testPub.Publish(messageString)
        # assert true
        time.sleep(2);

        finalSubMessageCount = testSub.GetNumberOfMessages()
        receivedMessage = testSub.GetLastMessageYAML()
        print(receivedMessage, initSubMessageCount, finalSubMessageCount)   # <-- UNABLE TO RECEIVE MESSAGE. 
        
        self.assertTrue(finalSubMessageCount - initSubMessageCount == 1)
        # assert that the recceived message contains the string message - since YAML
        self.assertTrue(messageString in receivedMessage)
        self.assertTrue(self.ros2Node.RemovePublisherNode('test_string_xkcd'))
        self.assertTrue(self.ros2Node.RemoveSubscriberNode('test_string_xkcd'))

    def test_pub_sub_deletion(self):
        testPub = self.ros2Node.CreateAndAddPublisher('vtkMRMLROS2PublisherStringNode', 'test_string_xkcd')
        testSub = self.ros2Node.CreateAndAddSubscriber('vtkMRMLROS2SubscriberStringNode', 'test_string_xkcd')

        self.assertTrue(self.ros2Node.RemovePublisherNode('test_string_xkcd'))
        self.assertFalse(self.ros2Node.RemovePublisherNode('test_string_xkcd'))
        self.assertFalse(self.ros2Node.RemovePublisherNode('random_name'))
        self.assertTrue(self.ros2Node.RemoveSubscriberNode('test_string_xkcd'))
        self.assertFalse(self.ros2Node.RemoveSubscriberNode('test_string_xkcd'))
        self.assertFalse(self.ros2Node.RemoveSubscriberNode('random_name'))

    def tearDown(self):
        # Kill the turtlesim node
        # self.ros2Node.Destroy()
        # time.sleep(1)
        pass

def run():
    suite  = unittest.TestLoader()
    # load all tests in this file
    suite = suite.loadTestsFromModule(sys.modules[__name__])
    unittest.TextTestRunner().run(suite)

if __name__ == '__main__':
    slicer_ros2_module_test()

# remove pub sub and check scene to make sure they are gone
# write a utility function to check if a node is running


"""
Notes: - Have not addressed yet.

1. Adding a ros2 node to the scene using AddNewNodeByClass after the widget is used to add ros2 node to the scene causes a crash. 
2. Destroying a ros2 node in the scene causes a crash. 
3. Creating publisher or subscriber twice leads to variable to the node being null on python terminal

"""
