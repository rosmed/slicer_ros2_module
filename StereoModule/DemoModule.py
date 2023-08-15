
import os
import slicer
import vtk
import numpy as np
from slicer.ScriptedLoadableModule import ScriptedLoadableModule, ScriptedLoadableModuleWidget, ScriptedLoadableModuleLogic
from lib.utils import createCustomLayout


class DemoModule(ScriptedLoadableModule):
    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        parent.title = "Stereo Control"
        parent.categories = ["IGT"]
        parent.dependencies = []
        parent.contributors = ["Aravind Sunil Kumar"]
        parent.helpText = """
        This is a simple module for controlling the stereo cameras using a robotic arm. Primary use case is for medical robotics.
        """
        parent.acknowledgementText = """
        Lorem ipsum dolor sit amet, consectetur adipiscing elit.
        """

class DemoModuleLogic(ScriptedLoadableModuleLogic):
    def __init__(self):
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
        
    def helper_print_matrix(self, matrix):
        for i in range(4):
            for j in range(4):
                print(f"{matrix.GetElement(i,j):.2f}", end = " ")
            print()

    def defineOffset(self, xOff = 0, yOff = 0, zOff =0):
        self.offset = np.array([xOff, yOff, zOff])
        self.leftCameraOffset = np.array([xOff, yOff, zOff])
        self.rightCameraOffset = np.array([-xOff, -yOff, -zOff])

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

    def displaceCamera(self, displacementRotationMatrix4x4, positionDisplacementVector):
        positionDisplacementMatrix = vtk.vtkMatrix4x4()

        leftCameraCurrentTransform, focal_disp_magnitude_left = self.GetCameraTransform(self.cameraNode1)
        rightCameraCurrentTransform, focal_disp_magnitude_right = self.GetCameraTransform(self.cameraNode2)

        currentLeftCameraPosition = self.extractPositionFromTransform(leftCameraCurrentTransform)
        currentRightCameraPosition = self.extractPositionFromTransform(rightCameraCurrentTransform)

        currentCentralCameraPosition = [(left + right)/2 for left, right in zip(currentLeftCameraPosition, currentRightCameraPosition)]

        centralCameraCurrentTransform = vtk.vtkMatrix4x4()
        centralCameraCurrentTransform.DeepCopy(leftCameraCurrentTransform)

        for i in range(3):
            centralCameraCurrentTransform.SetElement(i, 3, 0)
            positionDisplacementMatrix.SetElement(i, 3, positionDisplacementVector[i])

        newPositionDisplacementMatrix = self.multiplyMatrices(centralCameraCurrentTransform, positionDisplacementMatrix)
        centralCameraNextTransform = self.multiplyMatrices(centralCameraCurrentTransform, displacementRotationMatrix4x4)

        positionDisplacementVector = self.extractPositionFromTransform(newPositionDisplacementMatrix)

        for i, pos in enumerate(positionDisplacementVector):
            centralCameraNextTransform.SetElement(i, 3, pos + currentCentralCameraPosition[i])

        leftCameraNextTransform = self.multiplyMatrices(centralCameraNextTransform, self.transformLeftCamera)
        rightCameraNextTransform = self.multiplyMatrices(centralCameraNextTransform, self.transformRightCamera)

        self.SetCameraTransform(self.cameraNode1, leftCameraNextTransform, focal_disp_magnitude_left)
        self.SetCameraTransform(self.cameraNode2, rightCameraNextTransform, focal_disp_magnitude_right)

        self.leftCameraSittingTransform.SetMatrixTransformToParent(leftCameraNextTransform)
        self.rightCameraSittingTransform.SetMatrixTransformToParent(rightCameraNextTransform)

    def setup(self):
        ros2Node = slicer.mrmlScene.GetFirstNodeByName('ros2:node:slicer')
        self.lookupNode = ros2Node.CreateAndAddTf2LookupNode(
            "MTMR_base", "MTMR")
        self.lookupNodeID = self.lookupNode.GetID()
        self.scale_factor = 4

        self.buttonSubscriber = ros2Node.CreateAndAddSubscriberNode(
            "vtkMRMLROS2SubscriberJoyNode", "/console/camera")
        
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

        
    def GetCameraTransform(self, cameraNode):
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

        position = [transform.GetElement(0, 3), transform.GetElement(1, 3), transform.GetElement(2, 3)]
        
        yy = [transform.GetElement(0, 1), transform.GetElement(1, 1), transform.GetElement(2, 1)]
        focalDisplacement = np.array(yy) * magnitude
        focalPoint = np.array(position) + focalDisplacement

        viewUp = [transform.GetElement(0, 2), transform.GetElement(1, 2), transform.GetElement(2, 2)]

        cameraNode.SetPosition(position)
        cameraNode.SetFocalPoint(focalPoint)
        cameraNode.SetViewUp(viewUp)

    def _buttonCallback(self, caller, event):
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
        self.currentControllerTransform.DeepCopy(
            self.lookupNode.GetMatrixTransformToParent())
        self.moveCamera = True
        if self.startFlag:
            self.helperLogCameraStartState()
            self.startFlag = False

    def stopCameraControl(self):
        self.moveCamera = False

    def _callback(self, caller, event):
        if self.moveCamera:
            self.nextControllerTransform = caller.GetMatrixTransformToParent()
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


class DemoModuleWidget(ScriptedLoadableModuleWidget):
    def setup(self):
        ScriptedLoadableModuleWidget.setup(self)
        
        # Load the UI file
        uiFilePath = os.path.join(os.path.dirname(__file__), 'Resources/UI/DemoModule.ui')
        self.ui = slicer.util.loadUI(uiFilePath)
        self.layout.addWidget(self.ui)
        self.window = None
        
        # Connect the button to a function
        self.ui.demoButton.clicked.connect(self.onDemoButtonClick)

    def onDemoButtonClick(self):
        if self.window:
            self.window.close()

        self.window = createCustomLayout([100,100],[1280*2,720])
        self.window.show()

        self.logic = DemoModuleLogic()
        self.logic.setup()
        self.logic.defineOffset(-5, 0, 0)
        self.logic.ResetCameraPosition(0,-200,0)
        
        self.logic.displaceCamera(vtk.vtkMatrix4x4(), [0,0,0])
        self.logic.displaceCamera(vtk.vtkMatrix4x4(), [0,0,0])





