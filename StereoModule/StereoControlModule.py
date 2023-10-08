
import os
import slicer
import vtk
import numpy as np
from slicer.ScriptedLoadableModule import ScriptedLoadableModule, ScriptedLoadableModuleWidget, ScriptedLoadableModuleLogic
from lib.utils import *
try:
    import yaml
except:
    pip_install('pyyaml')
    import yaml


class StereoControlModule(ScriptedLoadableModule):
    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        parent.title = "Stereo Control"
        parent.categories = ["IGT"]
        parent.dependencies = ["ROS2"]
        parent.contributors = ["Aravind Sunil Kumar"]
        parent.helpText = """
        This is a simple module for controlling the stereo cameras using a robotic arm. Primary use case is for medical robotics.
        """
        parent.acknowledgementText = """
        Lorem ipsum dolor sit amet, consectetur adipiscing elit.
        """

class StereoControlModuleLogic(ScriptedLoadableModuleLogic):
    def __init__(self):
        """
        The above function initializes various variables and objects related to the layout manager, 3D
        widgets, view nodes, camera nodes, and transforms.
        """
        self.layoutManager = slicer.app.layoutManager()
        self.threeDWidget1 = self.layoutManager.threeDWidget(0)
        self.threeDWidget2 = self.layoutManager.threeDWidget(1)
        # Get the view nodes from the 3D widgets
        self.viewNode1 = self.threeDWidget1.mrmlViewNode()
        self.viewNode2 = self.threeDWidget2.mrmlViewNode()
        # Get the active camera nodes for both views
        self.cameraNode1 = slicer.modules.cameras.logic(
            ).GetViewActiveCameraNode(self.viewNode1)
        self.cameraNode2 = slicer.modules.cameras.logic(
            ).GetViewActiveCameraNode(self.viewNode2)
        # These are the transforms that are used to store state of the controller arm
        self.currentControllerTransform = vtk.vtkMatrix4x4()
        self.nextControllerTransform = vtk.vtkMatrix4x4()

        
    def helper_print_matrix(self, matrix):
        for i in range(4):
            for j in range(4):
                print(f"{matrix.GetElement(i,j):.2f}", end = " ")
            print()

    def defineOffset(self, xOff = 0, yOff = 0, zOff =0):
        """
        The `defineOffset` function sets the offset values for the left and right cameras, and creates
        transformation matrices for each camera.
        
        :param xOff: The x offset is the amount by which the camera is shifted horizontally from its
        original position. A positive value will shift the camera to the right, while a negative value will
        shift it to the left, defaults to 0 (optional)
        :param yOff: The parameter `yOff` represents the offset in the y-direction. It determines the
        distance by which the cameras are shifted vertically from their default position, defaults to 0
        (optional)
        :param zOff: The parameter `zOff` represents the offset in the z-direction. It determines the
        distance between the cameras and the object being viewed, defaults to 0 (optional)
        """
        self.offset = np.array([xOff, yOff, zOff])
        self.leftCameraOffset = np.array([xOff, yOff, zOff])
        self.rightCameraOffset = np.array([-xOff, -yOff, -zOff])

        # These are transformation matrices that would incorporate the offset values into the camera transforms
        self.transformLeftCamera = vtk.vtkMatrix4x4()
        self.transformRightCamera = vtk.vtkMatrix4x4()

        self.transformLeftCamera.SetElement(0, 3, self.leftCameraOffset[0])
        self.transformLeftCamera.SetElement(1, 3, self.leftCameraOffset[1])
        self.transformLeftCamera.SetElement(2, 3, self.leftCameraOffset[2])

        self.transformRightCamera.SetElement(0, 3, self.rightCameraOffset[0])
        self.transformRightCamera.SetElement(1, 3, self.rightCameraOffset[1])
        self.transformRightCamera.SetElement(2, 3, self.rightCameraOffset[2])
  
    def extractPositionFromTransform(self, cameraTransform):
        return [cameraTransform.GetElement(i, 3) for i in range(3)]

    def multiplyMatrices(self, mat1, mat2):
        result = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Multiply4x4(mat1, mat2, result)
        return result

    def displaceCamera(self, displacementRotationMatrix4x4, positionDisplacementVector): #TODO: position, matrix also make it 3x3 matrix
        """
        This function updates the position and orientation of two cameras based on a
        displacement rotation matrix and a position displacement vector.
        
        :param displacementRotationMatrix4x4: The `displacementRotationMatrix4x4` parameter is a vtk 4x4 matrix
        representing the rotation transformation that needs to be applied to the camera. 
        :param positionDisplacementVector: The `positionDisplacementVector` is a 3D vector that represents
        the displacement of the camera position.
        """
        # This variable is used to store the position displacement vector as a vtk 4x4 matrix
        positionDisplacementMatrix = vtk.vtkMatrix4x4()

        # Get the current camera transforms and focal displacement magnitudes. The focal displacement magnitude 
        # is the distance between the camera position and the focal point. And it is necessary to maintain this/scale this
        # distance when displacing the camera.
        leftCameraCurrentTransform, focal_disp_magnitude_left = self.GetCameraTransform(self.cameraNode1) #TODO: fix snake case
        rightCameraCurrentTransform, focal_disp_magnitude_right = self.GetCameraTransform(self.cameraNode2)

        # TODO: Create another class called StereoCamera for encapsulating the camera nodes and setup with just central processing happening here.
   
        # Extract the camera positions from the transforms
        currentLeftCameraPosition = self.extractPositionFromTransform(leftCameraCurrentTransform)
        currentRightCameraPosition = self.extractPositionFromTransform(rightCameraCurrentTransform)

        # We do all transforms relative to a hypothetical central camera position.
        # This is done to simplify the calculations.

        # Calculate the central camera position by taking the average of the left and right camera positions
        currentCentralCameraPosition = [(left + right)/2 for left, right in zip(currentLeftCameraPosition, currentRightCameraPosition)]

        # Create a copy of the left camera transform and set the translation components to 0
        # Since there is no relative rotation between the left and right cameras, we can use the rotation component 
        # of either camera to calculate the central camera's rotation component
        centralCameraCurrentTransform = vtk.vtkMatrix4x4()
        centralCameraCurrentTransform.DeepCopy(leftCameraCurrentTransform) 
        for i in range(3):
            centralCameraCurrentTransform.SetElement(i, 3, 0) 
            positionDisplacementMatrix.SetElement(i, 3, positionDisplacementVector[i])

        # Calculate the new position displacement matrix by multiplying the central camera's current transform with the position displacement matrix
        newPositionDisplacementMatrix = self.multiplyMatrices(centralCameraCurrentTransform, positionDisplacementMatrix)
        # Calculate the new central camera transform by multiplying the central camera's current transform with the displacement rotation matrix
        centralCameraNextTransform = self.multiplyMatrices(centralCameraCurrentTransform, displacementRotationMatrix4x4)
        # Extract the position displacement vector from the new position displacement matrix
        positionDisplacementVector = self.extractPositionFromTransform(newPositionDisplacementMatrix)
        # Add the position displacement vector to the current central camera position to get the new central camera position
        for i, pos in enumerate(positionDisplacementVector):
            centralCameraNextTransform.SetElement(i, 3, pos + currentCentralCameraPosition[i])
        # Calculate the new left and right camera transforms by multiplying the new central camera transform with the left and right camera transforms
        leftCameraNextTransform = self.multiplyMatrices(centralCameraNextTransform, self.transformLeftCamera)
        rightCameraNextTransform = self.multiplyMatrices(centralCameraNextTransform, self.transformRightCamera)
        # Set the new camera transforms
        self.SetCameraTransform(self.cameraNode1, leftCameraNextTransform, focal_disp_magnitude_left)
        self.SetCameraTransform(self.cameraNode2, rightCameraNextTransform, focal_disp_magnitude_right)
        # Set the new sitting transforms, they are used to visualize the camera positions in the 3D view
        self.leftCameraSittingTransform.SetMatrixTransformToParent(leftCameraNextTransform)
        self.rightCameraSittingTransform.SetMatrixTransformToParent(rightCameraNextTransform)

    def setup(self):
        """
        The setup function creates and initializes various nodes and variables for a ROS2 node in Slicer,
        including a TF2 lookup node, a subscriber node, and camera transform nodes.
        """
        ros2Node = slicer.mrmlScene.GetFirstNodeByName('ros2:node:slicer')
        self.lookupNode = ros2Node.CreateAndAddTf2LookupNode(
            "MTMR_base", "MTMR")
        self.lookupNodeID = self.lookupNode.GetID()
        self.scale_factor = 4

        # Create a subscriber node to receive messages from the ROS2 topic
        # this receives the button state from the controller
        self.buttonSubscriber = ros2Node.CreateAndAddSubscriberNode(
            "vtkMRMLROS2SubscriberJoyNode", "/console/camera")
        
        # Sitting transforms are used to visualize the camera positions in the 3D view
        if not slicer.mrmlScene.GetFirstNodeByName("leftCameraSittingTransform"):
            self.leftCameraSittingTransform = slicer.vtkMRMLLinearTransformNode()
            self.leftCameraSittingTransform.SetName("leftCameraSittingTransform")
            slicer.mrmlScene.AddNode(self.leftCameraSittingTransform)

        if not slicer.mrmlScene.GetFirstNodeByName("rightCameraSittingTransform"):
            self.rightCameraSittingTransform = slicer.vtkMRMLLinearTransformNode()
            self.rightCameraSittingTransform.SetName("rightCameraSittingTransform")
            slicer.mrmlScene.AddNode(self.rightCameraSittingTransform)

        self.moveCamera = False
        self.startFlag = False

        # Add observers to the subscriber node and the lookup node
        # The callback function is called whenever the node receives a message/event
        self.buttonObserverID = self.buttonSubscriber.AddObserver(
            "ModifiedEvent", self._buttonCallback)
        observerId = self.lookupNode.AddObserver(
            slicer.vtkMRMLTransformNode.TransformModifiedEvent, self._callback)

    def ResetCameraPosition(self, xDisp = 0, yDisp = 0, zDisp = 0):
        offset = self.offset
        self.cameraNode1.SetPosition(xDisp + self.offset[0], yDisp + self.offset[1], zDisp + self.offset[2])
        self.cameraNode1.SetFocalPoint(0, 0, 0)
        self.cameraNode1.SetViewUp(0, 0, 1)

        self.cameraNode2.SetPosition(xDisp - self.offset[0], yDisp - self.offset[1], zDisp - self.offset[2])
        # self.cameraNode2.SetPosition(0, yDisp - self.offset[1], 0)

        self.cameraNode2.SetFocalPoint(0, 0, 0)
        self.cameraNode2.SetViewUp(0, 0, 1)

        self.logic.DisplaceCamera(vtk.vtkMatrix4x4(), [0.0,0.0,0.0])


    #  TODO: Create a mono camera and stereo camera has two mono cameras. 
    def GetCameraTransform(self, cameraNode):
        """
        The function `GetCameraTransform` calculates the camera transform matrix and magnitude based on the
        position, focal point, and view up vectors of a camera node. This is required as 3D Slicer does not
        provide a function to get the camera transform matrix directly. 

        Z is View Up
        Y is Focal Direction
        X is Left to Right
        
        :param cameraNode: The `cameraNode` parameter is an object representing a camera in a 3D scene. It
        contains information about the camera's position, focal point, and view up vector
        :return: the camera transform matrix and the magnitude of the focal displacement.
        """
        position = cameraNode.GetPosition()
        focalPoint = cameraNode.GetFocalPoint()
        viewUp = cameraNode.GetViewUp()

        focalDisplacement = np.array(focalPoint) - np.array(position)
        magnitude = np.linalg.norm(focalDisplacement)

        zz = viewUp
        zz /= np.linalg.norm(zz)  # Normalize

        yy = focalDisplacement
        yy /= np.linalg.norm(yy)  # Normalize

        xx = np.cross(yy, zz)
        xx /= np.linalg.norm(xx)  # Normalize

        cameraTransform = vtk.vtkMatrix4x4()
        cameraTransform.Identity()
        for i in range(3):
            cameraTransform.SetElement(i, 0, xx[i])
            cameraTransform.SetElement(i, 1, yy[i])
            cameraTransform.SetElement(i, 2, zz[i])
            cameraTransform.SetElement(i, 3, position[i])

        return cameraTransform, magnitude
    
    def SetCameraTransform(self, cameraNode, transform, magnitude):
        """
        The function SetCameraTransform sets the position, focal point, and view up of a camera node based
        on a given transform and magnitude.
        
        :param cameraNode: The cameraNode parameter is an object representing the camera in the scene. It is
        used to set the position, focal point, and view up of the camera
        :param transform: The "transform" parameter is a 4x4 vtk matrix that represents the transformation of
        the camera. 
        :param magnitude: The magnitude parameter represents the distance or displacement from the camera's
        position to the focal point. 
        """

        position = [transform.GetElement(0, 3), transform.GetElement(1, 3), transform.GetElement(2, 3)]
        
        yy = [transform.GetElement(0, 1), transform.GetElement(1, 1), transform.GetElement(2, 1)]
        focalDisplacement = np.array(yy) * magnitude
        focalPoint = np.array(position) + focalDisplacement

        viewUp = [transform.GetElement(0, 2), transform.GetElement(1, 2), transform.GetElement(2, 2)]

        cameraNode.SetPosition(position)
        cameraNode.SetFocalPoint(focalPoint)
        cameraNode.SetViewUp(viewUp)

    def _buttonCallback(self, caller, event):
        """
        The function checks the value of a button and performs different actions based on
        its value.
        
        :param caller: The `caller` parameter is the object that triggered the callback function. 
        :param event: The "event" parameter is used to capture the event
        that triggered the callback. 
        """
        msg_yaml = caller.GetLastMessageYAML()
        msg = yaml.load(msg_yaml, Loader=yaml.FullLoader)
        button = msg['buttons'][0]

        if button == 1 and self.moveCamera == False:
            self.startCameraControl()
        elif button == 0 and self.moveCamera == True:
            self.stopCameraControl()
        else:
            print(f"Received button value: {button}")

    def helperLogCameraStartState(self):
        self.leftCameraInitialTransform, self.focal_disp_magnitude_left_initial = self.GetCameraTransform(self.cameraNode1)
        self.leftCameraInitialTransform = vtk.vtkMatrix4x4()

    def startCameraControl(self):
        self.currentControllerTransform = vtk.vtkMatrix4x4()
        # self.currentControllerTransform.DeepCopy(
        #     self.lookupNode.GetMatrixTransformToParent())
        self.lookupNode.GetMatrixTransformToParent(self.currentControllerTransform)
        self.moveCamera = True
        if self.startFlag:
            self.helperLogCameraStartState()
            self.startFlag = False

    def stopCameraControl(self):
        self.moveCamera = False

    def _callback(self, caller, event):
        """
        The `_callback` function updates the camera position based on the movement of a controller.
        
        :param caller: The "caller" parameter is an object that is calling the _callback method. 
        :param event: The "event" parameter is an object that represents the event that triggered the
        callback function. 
        """
        if self.moveCamera:
            caller.GetMatrixTransformToParent(self.nextControllerTransform)
            displacement, positionDisplacementVector = findDisplacementTransform(
                self.currentControllerTransform, self.nextControllerTransform, self.scale_factor)
            self.displaceCamera(displacement, positionDisplacementVector)
            # copy contents of nextControllerTransform into currentControllerTransform they are vtkMatrix4x4 objects
            self.currentControllerTransform.DeepCopy(self.nextControllerTransform)

    def debugCameraPosition(self, xDisp, yDisp, zDisp, xOff, yOff, zOff):
        self.defineOffset(xOff, yOff, zOff)
        self.ResetCameraPosition(xDisp, yDisp, zDisp)
        self.displaceCamera(vtk.vtkMatrix4x4(), [0,0,0])
        self.displaceCamera(vtk.vtkMatrix4x4(), [0,0,0])
    
    def createStereoLayout(self):
        # Move your stereo layout creation code here
        print("Logic called!")


class StereoControlModuleWidget(ScriptedLoadableModuleWidget):
    def setup(self):
        ScriptedLoadableModuleWidget.setup(self)
        
        # Load the UI file
        uiFilePath = os.path.join(os.path.dirname(__file__), 'Resources/UI/StereoControlModule.ui')
        self.ui = slicer.util.loadUI(uiFilePath)
        self.layout.addWidget(self.ui)
        self.window = None
        
        # Connect the button to a function
        self.ui.demoButton.clicked.connect(self.onDemoButtonClick)

    def onDemoButtonClick(self):
        """
        The function `onDemoButtonClick` creates a custom layout window, sets up a stereo control module
        logic, defines an offset, resets the camera position, and displaces the camera.
        """
        if self.window:
            self.window.close()

        # Set position and resolution of the window
        self.window = createCustomLayout([100,100],[1280*2,720])
        self.window.show()

        self.logic = StereoControlModuleLogic()
        self.logic.Setup()
        self.logic.DefineOffset(-5.0, 0.0, 0.0) # Setting offset between left, right cameras to the center of the two
        # TODO: it should only accpt 2* del X. Call it set X basiline and get rid of the x,y arguments
        # self.logic.SetBaseline(10) # TODO: In the documentation, the coordinate system needs to be defined.

        self.logic.ResetCameraPosition(0.0,-200.0,0.0) #TODO: SetAbsoluteCameraPosition and add VTK matrix as a parameter too
        # TODO: email laura about measured_cp from 3D Slicer object. (3D markers)
        
        

s



