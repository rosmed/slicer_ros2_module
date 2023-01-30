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

1. Adding a ros2 node to the scene using AddNewNodeByClass after the widget is used to add ros2 node to the scene causes a crash. This is because the widget is already using the ros2 node. The widget should be able to handle this case and not crash.
2. Destroying a ros2 node in the scene causes a crash. 


"""


