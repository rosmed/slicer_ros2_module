import logging
import os

import vtk

import slicer
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
import unittest
import subprocess
import logging
import sys
try:
    import psutil
except:
    slicer.util.pip_install('psutil')

#
# ROS2Tests
#

class ROS2Tests(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = "ROS2Tests"
        self.parent.categories = ["Examples"] # FIXME: replace with IGT
        self.parent.dependencies = ['ROS2']
        self.parent.contributors = ["Aravind Kumar (JHU)"]
        self.parent.helpText = """
This module is used to test the SlicerROS2 module.  It doesn't provide a UI and can be used in the Python interpreter using:
tests = slicer.util.getModuleLogic('ROS2Tests')
tests.run()
"""
        # TODO: replace with organization, grant and thanks
        self.parent.acknowledgementText = """
This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab,
and Steve Pieper, Isomics, Inc. and was partially funded by NIH grant 3P41RR013218-12S1.
"""


class ROS2TestsWidget(ScriptedLoadableModuleWidget):
    """Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent=None):
        """
        Called when the user opens the module the first time and the widget is initialized.
        """
        ScriptedLoadableModuleWidget.__init__(self, parent)
        self.logic = None

    def setup(self):
        """
        Called when the user opens the module the first time and the widget is initialized.
        """
        ScriptedLoadableModuleWidget.setup(self)

        # Load widget from .ui file (created by Qt Designer).
        # Additional widgets can be instantiated manually and added to self.layout.
        # uiWidget = slicer.util.loadUI(self.resourcePath('UI/ROS2Tests.ui'))
        # self.layout.addWidget(uiWidget)
        # self.ui = slicer.util.childWidgetVariables(uiWidget)

        # Create logic class. Logic implements all computations that should be possible to run
        # in batch mode, without a graphical user interface.
        self.logic = ROS2TestsLogic()


#
# ROS2TestsLogic
#

class TestObserverSubscriber:
    def __init__(self):
        self.lastMessageYAML = ""
        self.counter = 0

    def Callback(self, caller, event):
        self.lastMessageYAML = caller.GetLastMessageYAML()
        self.counter += 1

class TestObserverTf2Lookup:
    def __init__(self):
        self.counter = 0

    def Callback(self, caller, event):
        self.counter += 1
        self.lastTransform = caller.GetMatrixTransformToParent()


class ROS2TestsLogic(ScriptedLoadableModuleLogic):
    """This class should implement all the actual
    computation done by your module.  The interface
    should be such that other python code can import
    this class and make use of the functionality without
    requiring an instance of the Widget.
    Uses ScriptedLoadableModuleLogic base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    ENVIRONMENT_CORRECTION = "export PYTHONPATH=/opt/ros/galactic/lib/python3.8/site-packages; export PYTHONHOME=; "
    ROS2_EXEC = ENVIRONMENT_CORRECTION + "python3.8 /opt/ros/galactic/bin/ros2 "

    def __init__(self):
        """
        Called when the logic class is instantiated. Can be used for initializing member variables.
        """
        ScriptedLoadableModuleLogic.__init__(self)

    @classmethod
    def spin_some(self):
        ros2Logic = slicer.util.getModuleLogic('ROS2')
        for i in range(3):
            ros2Logic.Spin()

    @classmethod
    def run_ros2_cli_command_blocking(self, command):
        ros2_process = subprocess.Popen(
            ROS2TestsLogic.ROS2_EXEC + command,
            shell=True,
            preexec_fn=os.setsid,
        )
        ros2_process.wait()
        return ros2_process

    @classmethod
    def run_ros2_cli_command_non_blocking(self, command):
        ros2_process = subprocess.Popen(
            ROS2TestsLogic.ROS2_EXEC + command,
            shell=True,
            preexec_fn=os.setsid,
        )
        return ros2_process

    @classmethod
    def kill_subprocess(self, proc):
        os.killpg(os.getpgid(proc.pid), subprocess.signal.SIGINT)

    @classmethod
    def check_ros2_node_running(self, nodeName):
        # Check if the turtlesim node is running by checking the rosnode list
        nodes = (
            subprocess.check_output(
                ROS2TestsLogic.ROS2_EXEC + "node list",
                shell=True,
            )
            .decode("utf-8")
            .split("\n")
        )
        # Assert that the turtlesim node is in the list of running nodes
        return nodeName in nodes

        # It creates a turtlesim node, checks if it's running, and then kills it
    class TestTurtlesimNode(unittest.TestCase):
        def setUp(self):
            self.create_turtlesim_node_process = ROS2TestsLogic.run_ros2_cli_command_non_blocking("run turtlesim turtlesim_node")

        def test_turtlesim_node_create_and_destroy(self):
            print("\nTesting creation and destruction of turtlesim node - Starting..")
            # Check if the turtlesim node is running by checking the rosnode list
            self.assertTrue(ROS2TestsLogic.check_ros2_node_running("/turtlesim"), "Turtlesim node not running")
            print("Testing creation and destruction of turtlesim node - Done")

        def tearDown(self):
            # Kill the turtlesim node
            ROS2TestsLogic.kill_subprocess(self.create_turtlesim_node_process)
            self.assertFalse(ROS2TestsLogic.check_ros2_node_running("/turtlesim"), "Turtlesim node still running")

    # It creates a ROS2 node, adds a publisher and subscriber to it, and publishes a message
    class TestCreateAndAddPubSub(unittest.TestCase):
        def setUp(self):
            print("\nCreating ROS2 node for pub sub tests..")
            self.ros2Node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLROS2NodeNode")
            self.ros2Node.Create("testNode")
            self.testObs = TestObserverSubscriber()
            ROS2TestsLogic.spin_some()

        def create_pub_sub(self, type): # TODO: Keep just type to append to vtkMRMLROS2Publisher/Subscriber?
            self.topic = "slicer_test_" + type.lower()
            self.pubType = "vtkMRMLROS2Publisher" + type + "Node"
            self.subType = "vtkMRMLROS2Subscriber" + type + "Node"
            self.testPub = self.ros2Node.CreateAndAddPublisherNode(self.pubType, self.topic)
            self.testSub = self.ros2Node.CreateAndAddSubscriberNode(self.subType, self.topic)
            self.testSub.AddObserver("ModifiedEvent", self.testObs.Callback)
            ROS2TestsLogic.spin_some()

        def generic_assertions(self, initSubMessageCount):
            ROS2TestsLogic.spin_some()

            finalSubMessageCount = self.testSub.GetNumberOfMessages()
            self.assertTrue(finalSubMessageCount - initSubMessageCount == 1, "Message not received")

            self.assertTrue(self.testSub.GetLastMessageYAML() == self.testObs.lastMessageYAML,
                            "Message not received correctly by observer")
            self.assertTrue(self.testObs.counter == 1,
                            "Observer number of calls incorrect")

        def delete_pub_sub(self):
            self.assertTrue(self.ros2Node.RemoveAndDeletePublisherNode(self.topic), "Publisher not deleted")
            self.assertTrue(self.ros2Node.RemoveAndDeleteSubscriberNode(self.topic), "Subscriber not deleted")
            return

        def test_create_and_add_pub_sub_string(self):
            print("\nTesting creation and working of publisher and subscriber - Starting..")
            self.create_pub_sub("String")

            initSubMessageCount = self.testSub.GetNumberOfMessages()
            sentString = "xkcd"
            self.testPub.Publish(sentString)

            self.generic_assertions(initSubMessageCount)

            receivedString = self.testSub.GetLastMessage()
            self.assertTrue(sentString == receivedString, "Message not received correctly")

            self.delete_pub_sub()
            print("Testing creation and working of publisher and subscriber - Done")

        def test_create_and_add_pub_sub_matrix(self):
            print("\nTesting creation and working of publisher and subscriber - Starting..")
            self.create_pub_sub("PoseStamped")

            initSubMessageCount = self.testSub.GetNumberOfMessages()
            sentMatrix = vtk.vtkMatrix4x4()
            sentMatrix.SetElement(2, 3, 3.1415)
            self.testPub.Publish(sentMatrix)

            self.generic_assertions(initSubMessageCount)

            receivedMatrix = self.testSub.GetLastMessage()
            self.assertTrue(sentMatrix.GetElement(2, 3) == receivedMatrix.GetElement(2, 3),
                            "Message not received correctly")

            self.delete_pub_sub()
            print("Testing creation and working of publisher and subscriber - Done")

        def test_create_and_add_pub_sub_double(self):
            print("\nTesting creation and working of publisher and subscriber - Starting..")
            self.create_pub_sub("Double")

            initSubMessageCount = self.testSub.GetNumberOfMessages()
            sentDouble = 3.1415
            self.testPub.Publish(sentDouble)
            ROS2TestsLogic.spin_some()

            self.generic_assertions(initSubMessageCount)

            receivedDouble = self.testSub.GetLastMessage()
            self.assertTrue(sentDouble == receivedDouble, "Message not received correctly")

            self.delete_pub_sub()
            print("Testing creation and working of publisher and subscriber - Done")

        def test_create_and_add_pub_sub_int_array(self):
            print("\nTesting creation and working of publisher and subscriber - Starting..")
            self.create_pub_sub("IntArray")

            initSubMessageCount = self.testSub.GetNumberOfMessages()
            sentIntArray = [1, 2, 3, 4, 5]
            sentIntVtkArray = vtk.vtkIntArray()
            sentIntVtkArray.SetNumberOfValues(len(sentIntArray))
            for i in range(len(sentIntArray)):
                sentIntVtkArray.SetValue(i, sentIntArray[i])
            self.testPub.Publish(sentIntVtkArray)

            self.generic_assertions(initSubMessageCount)

            receivedIntArray = self.testSub.GetLastMessage()
            # check that sentIntVtkArray and receivedIntArray are the same
            self.assertTrue(sentIntVtkArray.GetNumberOfValues() == receivedIntArray.GetNumberOfValues(), "Message not received correctly")
            for i in range(sentIntVtkArray.GetNumberOfValues()):
                self.assertTrue(sentIntVtkArray.GetValue(i) == receivedIntArray.GetValue(i), "Message not received correctly")

            self.delete_pub_sub()
            print("Testing creation and working of publisher and subscriber - Done")

        def test_create_and_add_pub_sub_double_array(self):
            print("\nTesting creation and working of publisher and subscriber - Starting..")
            self.create_pub_sub("DoubleArray")

            initSubMessageCount = self.testSub.GetNumberOfMessages()
            sentDoubleArray = [1.1, 2.2, 3.3, 4.4, 5.5]
            sentDoubleVtkArray = vtk.vtkDoubleArray()
            sentDoubleVtkArray.SetNumberOfValues(len(sentDoubleArray))
            for i in range(len(sentDoubleArray)):
                sentDoubleVtkArray.SetValue(i, sentDoubleArray[i])
            self.testPub.Publish(sentDoubleVtkArray)

            self.generic_assertions(initSubMessageCount)

            receivedDoubleArray = self.testSub.GetLastMessage()
            # check that sentDoubleVtkArray and receivedDoubleArray are the same
            self.assertTrue(sentDoubleVtkArray.GetNumberOfValues() == receivedDoubleArray.GetNumberOfValues(), "Message not received correctly")
            for i in range(sentDoubleVtkArray.GetNumberOfValues()):
                self.assertTrue(sentDoubleVtkArray.GetValue(i) == receivedDoubleArray.GetValue(i), "Message not received correctly")

            self.delete_pub_sub()
            print("Testing creation and working of publisher and subscriber - Done")

        def test_create_and_add_pub_sub_int_n_array(self):
            print("\nTesting creation and working of publisher and subscriber for N-array - Starting..")
            self.create_pub_sub("IntTable")
            initSubMessageCount = self.testSub.GetNumberOfMessages()

            arr1 = vtk.vtkIntArray()
            arr2 = vtk.vtkIntArray()
            arr1.SetNumberOfValues(3)
            arr2.SetNumberOfValues(3)
            for i in range(3):
                arr1.SetValue(i, 3*i) # 0, 3, 6
                arr2.SetValue(i, 4*i+1) # 1, 5, 9
            vtktable = vtk.vtkTable()
            vtktable.AddColumn(arr1)
            vtktable.AddColumn(arr2)

            self.testPub.Publish(vtktable)
            self.generic_assertions(initSubMessageCount)
            receivedVtkTable = self.testSub.GetLastMessage()
            # check that vtktable and receivedVtkTable are the same
            self.assertTrue(vtktable.GetNumberOfColumns() == receivedVtkTable.GetNumberOfColumns(), "Message not received correctly")
            self.assertTrue(vtktable.GetNumberOfRows() == receivedVtkTable.GetNumberOfRows(), "Message not received correctly")
            for i in range(vtktable.GetNumberOfColumns()):
                for j in range(vtktable.GetNumberOfRows()):
                    self.assertTrue(vtktable.GetValue(j, i) == receivedVtkTable.GetValue(j, i), "Message not received correctly")

            self.delete_pub_sub()
            print("Testing creation and working of publisher and subscriber - Done")

        def test_create_and_add_pub_sub_double_n_array(self):
            print("\nTesting creation and working of publisher and subscriber for N-array - Starting..")
            self.create_pub_sub("DoubleTable")
            initSubMessageCount = self.testSub.GetNumberOfMessages()

            arr1 = vtk.vtkDoubleArray()
            arr2 = vtk.vtkDoubleArray()
            arr1.SetNumberOfValues(3)
            arr2.SetNumberOfValues(3)
            for i in range(3):
                arr1.SetValue(i, 3.3*i) # 0, 3.3, 6.6
                arr2.SetValue(i, 4.4*i+1.1) # 1.1, 5.5, 9.9

            vtktable = vtk.vtkTable()
            vtktable.AddColumn(arr1)
            vtktable.AddColumn(arr2)

            self.testPub.Publish(vtktable)
            self.generic_assertions(initSubMessageCount)
            receivedVtkTable = self.testSub.GetLastMessage()
            # check that vtktable and receivedVtkTable are the same
            self.assertTrue(vtktable.GetNumberOfColumns() == receivedVtkTable.GetNumberOfColumns(), "Message not received correctly")
            self.assertTrue(vtktable.GetNumberOfRows() == receivedVtkTable.GetNumberOfRows(), "Message not received correctly")
            for i in range(vtktable.GetNumberOfColumns()):
                for j in range(vtktable.GetNumberOfRows()):
                    self.assertTrue(vtktable.GetValue(j, i) == receivedVtkTable.GetValue(j, i), "Message not received correctly")

            self.delete_pub_sub()
            print("Testing creation and working of publisher and subscriber - Done")

        def test_pub_sub_deletion(self):
            print("\nTesting deletion of publisher and subscriber - Starting..")
            testPub = self.ros2Node.CreateAndAddPublisherNode(
                "vtkMRMLROS2PublisherStringNode", "test_string_xkcd"
            )
            testSub = self.ros2Node.CreateAndAddSubscriberNode(
                "vtkMRMLROS2SubscriberStringNode", "test_string_xkcd"
            )

            # delete publisher which exists
            self.assertTrue(self.ros2Node.RemoveAndDeletePublisherNode("test_string_xkcd"), "Publisher which exists not removed")
            ROS2TestsLogic.spin_some()
            # delete publisher which used to exist
            self.assertFalse(self.ros2Node.RemoveAndDeletePublisherNode("test_string_xkcd"), "Publisher which used to exist removed")
            ROS2TestsLogic.spin_some()
            # delete publisher which never existed
            self.assertFalse(self.ros2Node.RemoveAndDeletePublisherNode("random_name"), "Publisher which never existed removed")
            ROS2TestsLogic.spin_some()
            # delete subscriber which exists
            self.assertTrue(self.ros2Node.RemoveAndDeleteSubscriberNode("test_string_xkcd"), "Subscriber which exists not removed")
            ROS2TestsLogic.spin_some()
            # delete subscriber which used to exist
            self.assertFalse(self.ros2Node.RemoveAndDeleteSubscriberNode("test_string_xkcd"), "Subscriber which used to exist removed")
            ROS2TestsLogic.spin_some()
            # delete subscriber which never existed
            self.assertFalse(self.ros2Node.RemoveAndDeleteSubscriberNode("random_name"), "Subscriber which never existed removed")
            ROS2TestsLogic.spin_some()
            print("Testing deletion of publisher and subscriber - Done")

        def tearDown(self):
            self.ros2Node.Destroy()
            ROS2TestsLogic.spin_some()

    class TestParameterNode(unittest.TestCase):
        def setUp(self):
            print("\nCreating ROS2 node for parameter tests..")
            self.ros2Node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLROS2NodeNode")
            self.ros2Node.Create("testNode")
            ROS2TestsLogic.spin_some()
            self.create_turtlesim_node_process = ROS2TestsLogic.run_ros2_cli_command_non_blocking("run turtlesim turtlesim_node")
            ROS2TestsLogic.spin_some()
            self.assertTrue(ROS2TestsLogic.check_ros2_node_running("/turtlesim"), "Turtlesim node not running")
            ROS2TestsLogic.spin_some()

        def test_parameter_monitoring(self):
            print("\nTesting creation and working of parameter node - Starting..")
            testParam = self.ros2Node.CreateAndAddParameterNode("/turtlesim")
            ROS2TestsLogic.spin_some()

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
                ROS2TestsLogic.spin_some()

            self.assertEqual(testParam.GetParameterType("background_r"), "integer", "Parameter type not integer")

            self.assertFalse(testParam.IsParameterSet("background_y", True), "Parameter type not empty")
            self.assertEqual(testParam.GetParameterType("background_y"), "", "Parameter type not empty")

            # Update parameter value
            change_param_value = ROS2TestsLogic.run_ros2_cli_command_blocking(
                "param set /turtlesim background_g 150"
            )

            while(testParam.IsParameterSet("background_g", True) == False):
                ROS2TestsLogic.spin_some()

            self.assertTrue(testParam.IsParameterSet("background_g"))
            self.assertEqual(testParam.GetParameterType("background_g"), "integer")
            self.assertEqual(testParam.GetParameterAsInteger("background_g"), 150)

            # delete parameter node
            self.assertTrue(self.ros2Node.RemoveAndDeleteParameterNode("/turtlesim"))
            print("Testing creation and working of parameter node - Done")

        def test_parameter_deletion(self):
            print("\nTesting deletion of parameter node - Starting..")
            testParam = self.ros2Node.CreateAndAddParameterNode("/turtlesim")
            ROS2TestsLogic.spin_some()

            # delete parameter node which exists
            self.assertTrue(self.ros2Node.RemoveAndDeleteParameterNode("/turtlesim"), "Failed to delete parameter node which exists")
            ROS2TestsLogic.spin_some()
            # delete parameter node which used to exist
            self.assertFalse(self.ros2Node.RemoveAndDeleteParameterNode("/turtlesim"), "Deleted parameter node which used to exist")
            ROS2TestsLogic.spin_some()
            # delete parameter node which never existed
            self.assertFalse(self.ros2Node.RemoveAndDeleteParameterNode("/random_name"), "Deleted parameter node which never existed")
            ROS2TestsLogic.spin_some()
            print("Testing deletion of parameter node - Done")

        def tearDown(self):
            ROS2TestsLogic.kill_subprocess(self.create_turtlesim_node_process)
            self.ros2Node.Destroy()
            ROS2TestsLogic.spin_some()
            print("ROS2 node destroyed")


    class TestTf2BroadcasterAndLookupNode(unittest.TestCase):
        def setUp(self):
            print("\nCreating ROS2 node to test Broadcaster Nodes..")
            self.ros2Node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLROS2NodeNode")
            self.ros2Node.Create("testNode")
            ROS2TestsLogic.spin_some()

        def test_broadcaster_functioning(self):
            broadcaster = self.ros2Node.CreateAndAddTf2BroadcasterNode("Parent", "Child")
            lookupNode = self.ros2Node.CreateAndAddTf2LookupNode("Parent", "Child")
            observer = TestObserverTf2Lookup()
            lookupNode.AddObserver(slicer.vtkMRMLTransformNode.TransformModifiedEvent, observer.Callback)
            # Broadcast a 4x4 matrix and confirm
            broadcastedMat = vtk.vtkMatrix4x4()
            broadcastedMat.SetElement(0,3,66) # Set a default value
            broadcaster.Broadcast(broadcastedMat)
            ROS2TestsLogic.spin_some()
            lookupMat = lookupNode.GetMatrixTransformToParent()
            self.assertEqual(lookupMat.GetElement(0,3), broadcastedMat.GetElement(0,3)) # maybe use assert almost equal
            self.assertEqual(observer.counter, 1)
            self.assertEqual(observer.lastTransform.GetElement(0,3), broadcastedMat.GetElement(0,3))

        def tearDown(self):
            pass
            # self.ros2Node.Destroy()


    def run(self):
        print('Running all tests...')

        suite = unittest.TestSuite()
        suite.addTest(unittest.makeSuite(ROS2TestsLogic.TestTurtlesimNode))
        suite.addTest(unittest.makeSuite(ROS2TestsLogic.TestCreateAndAddPubSub))
        suite.addTest(unittest.makeSuite(ROS2TestsLogic.TestParameterNode))
        suite.addTest(unittest.makeSuite(ROS2TestsLogic.TestTf2BroadcasterAndLookupNode))

        runner = unittest.TextTestRunner()
        runner.run(suite)


# tests = slicer.util.getModuleLogic('ROS2Tests')
# tests.run()

# ros2 = slicer.mrmlScene.GetFirstNodeByName('ros2:node:slicer')
# pub = ros2.CreateAndAddPublisherNode('vtkMRMLROS2PublisherIntTableNode','testpub2')
