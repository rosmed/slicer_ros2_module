import slicer
import vtk
import numpy as np
try:
    import yaml
except:
    pip_install('pyyaml')
    import yaml

from scipy.spatial.transform import Rotation as R


class StereoView:
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
        
        # self.leftCameraInitialTransform = vtk.vtkMatrix4x4()

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


def add_model_to_scene(path_to_model):
    model = slicer.util.loadModel(path_to_model)
    if not model:
        print("Error loading model")
        return


def createCustomLayout(position=[100, 100], resolution=[1920, 1080]):
    customLayout = """
    <layout type="vertical" >
        <item>
            <view class="vtkMRMLViewNode" singletontag="1"/>
        </item>
        <item>
            <view class="vtkMRMLViewNode" singletontag="2"/>
        </item>
        <item>
            <view class="vtkMRMLViewNode" singletontag="3"/>
        </item>
    </layout>
    """

    # Register the custom layout with the application layout manager
    layoutManager = slicer.app.layoutManager()
    customLayoutId = 100  # an arbitrary ID number for the custom layout
    layoutManager.layoutLogic().GetLayoutNode(
    ).AddLayoutDescription(customLayoutId, customLayout)
    # Set the application layout to the custom layout
    layoutManager.setLayout(customLayoutId)

    threeDWidget1 = layoutManager.threeDWidget(0)
    threeDWidget2 = layoutManager.threeDWidget(1)

    popupWindow = qt.QWidget()
    popupWindow.setWindowTitle("Test ABCD")
    popupWindow.setLayout(qt.QHBoxLayout())
    popupWindow.layout().addWidget(threeDWidget1)
    popupWindow.layout().addWidget(threeDWidget2)

    # Number of pixels from the left and top of the screen
    popupWindow.move(*position)
    popupWindow.resize(*resolution)

    return popupWindow

DEBUG_INTERPOLATION = 1

def findDisplacementTransform(startTransform, endTransform, scale_factor):
    displacementTransform = vtk.vtkMatrix4x4()

    # end = displacement * start
    # displacement = end * start^-1
    startTransformInverse = vtk.vtkMatrix4x4()
    vtk.vtkMatrix4x4.Invert(startTransform, startTransformInverse)
    vtk.vtkMatrix4x4.Multiply4x4(
        endTransform, startTransformInverse, displacementTransform)

    scale_factor = 0.3 ## TODO: clean this up

    positionDisplacementVector = [(endTransform.GetElement(
        i, 3) - startTransform.GetElement(i, 3)) * scale_factor for i in range(3)]

    # divide position by scale factor
    displacementTransform.SetElement(
        0, 3, 0)
    displacementTransform.SetElement(
        1, 3, 0)
    displacementTransform.SetElement(
        2, 3, 0)
    
    # displacementTransform = slerp_vtk_rotation_matrix(displacementTransform)
    displacementTransform = interpolate_vtk_matrix(displacementTransform, DEBUG_INTERPOLATION)

    return displacementTransform, positionDisplacementVector


def slerp(q0, q1, t):
    dot_product = np.dot(q0, q1)
    dot_product = np.clip(dot_product, -1.0, 1.0)
    theta = np.arccos(dot_product)
    slerp_q = (np.sin((1-t)*theta) / np.sin(theta)) * q0 + (np.sin(t*theta) / np.sin(theta)) * q1
    return slerp_q

def interpolate_vtk_matrix(vtk_matrix, t):
    # Convert vtkMatrix4x4 to numpy array
    matrix_array = np.array(vtk_matrix)

    matrix_array = np.array([[vtk_matrix.GetElement(i, j) for j in range(4)] for i in range(4)])

    # Extract 3x3 rotation matrix
    R_mat = matrix_array[0:3, 0:3]

    # Convert R to a quaternion
    rotation = R.from_matrix(R_mat)
    qR = rotation.as_quat()

    # Perform slerp between identity quaternion and qR
    identity_matrix = np.identity(3)
    identity_rotation = R.from_matrix(identity_matrix)
    identity_quaternion = identity_rotation.as_quat()
    qT = slerp(identity_quaternion, qR, t)

    # Convert interpolated quaternion back to rotation matrix
    if np.isclose(np.linalg.norm(qT), 0):
        intermediate_rotation = np.identity(3)
    else:
        intermediate_rotation = R.from_quat(qT)
    RT = intermediate_rotation.as_matrix()

    # Construct the resulting vtkMatrix4x4
    result_vtk_matrix = vtk.vtkMatrix4x4()
    for i in range(3):
        for j in range(3):
            result_vtk_matrix.SetElement(i, j, RT[i, j])

    return result_vtk_matrix

def getCentroidofModel(modelName):

    modelNode = slicer.util.getNode(modelName)
    polyData = modelNode.GetPolyData()
    numPoints = polyData.GetNumberOfPoints()
    sumCoords = np.array([0.0,0.0,0.0])
    for i in range(numPoints):
        point = [0.0,0.0,0.0]
        polyData.GetPoint(i, point)
        sumCoords += np.array(point)
    centroid = sumCoords / numPoints
    print("Centroid: ", centroid)


if __name__ == "__main__":

    # Create a custom layout
    popw = createCustomLayout(resolution=[1280, 720])
    popw.show()  # only works if called in the main function
    stereo = StereoView()
    # add_model_to_scene("TumorModel.vtk")
    stereo.setup()
    stereo.defineOffset(-10, 0, 0)
    stereo.ResetCameraPosition(0,-80,0)
    
    stereo.displaceCamera(vtk.vtkMatrix4x4(), [0,0,0])
    stereo.displaceCamera(vtk.vtkMatrix4x4(), [0,0,0])

# exec(open('layout_custom.py').read())
