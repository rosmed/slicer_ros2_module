import slicer
import vtk
import numpy as np
try:
    import yaml
except:
    pip_install('pyyaml')
    import yaml

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

    def displaceCamera(self, displacementRotationMatrix4x4, positionDisplacementVector):
        leftCameraCurrentTransform, focal_disp_magnitude = self.GetCameraTransform(self.cameraNode1)

        leftCameraNextTransform = vtk.vtkMatrix4x4()

        currentCameraPosition = [leftCameraCurrentTransform.GetElement(
            0, 3), leftCameraCurrentTransform.GetElement(1, 3), leftCameraCurrentTransform.GetElement(2, 3)]
        
        leftCameraCurrentTransform.SetElement(0, 3, 0)
        leftCameraCurrentTransform.SetElement(1, 3, 0)
        leftCameraCurrentTransform.SetElement(2, 3, 0)

        vtk.vtkMatrix4x4.Multiply4x4(
            displacementRotationMatrix4x4, leftCameraCurrentTransform, leftCameraNextTransform)
        
        leftCameraNextTransform.SetElement(
            0, 3,  positionDisplacementVector[0] + currentCameraPosition[0])
        leftCameraNextTransform.SetElement(
            1, 3,  positionDisplacementVector[1] + currentCameraPosition[1])
        leftCameraNextTransform.SetElement(
            2, 3,  positionDisplacementVector[2] + currentCameraPosition[2])
        
        self.SetCameraTransform(self.cameraNode1, leftCameraNextTransform, focal_disp_magnitude)
        self.SetCameraTransform(self.cameraNode2, leftCameraNextTransform, focal_disp_magnitude)

        self.leftCameraSittingTransform.SetMatrixTransformToParent(
            leftCameraNextTransform)

        self.rightCameraSittingTransform.SetMatrixTransformToParent(
            leftCameraNextTransform)

    def setup(self):
        ros2Node = slicer.mrmlScene.GetFirstNodeByName('ros2:node:slicer')
        self.lookupNode = ros2Node.CreateAndAddTf2LookupNode(
            "MTMR_base", "MTMR")
        self.lookupNodeID = self.lookupNode.GetID()
        self.scale_factor = 5

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

        self.buttonObserverID = self.buttonSubscriber.AddObserver(
            "ModifiedEvent", self._buttonCallback)
        observerId = self.lookupNode.AddObserver(
            slicer.vtkMRMLTransformNode.TransformModifiedEvent, self._callback)
        
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

    def startCameraControl(self):
        self.currentControllerTransform = vtk.vtkMatrix4x4()
        self.currentControllerTransform.DeepCopy(
            self.lookupNode.GetMatrixTransformToParent())
        self.moveCamera = True

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


def findDisplacementTransform(startTransform, endTransform, scale_factor):
    displacementTransform = vtk.vtkMatrix4x4()

    # end = displacement * start
    # displacement = end * start^-1
    startTransformInverse = vtk.vtkMatrix4x4()
    vtk.vtkMatrix4x4.Invert(startTransform, startTransformInverse)
    vtk.vtkMatrix4x4.Multiply4x4(
        endTransform, startTransformInverse, displacementTransform)

    scale_factor = 0.2

    positionDisplacementVector = [(endTransform.GetElement(
        i, 3) - startTransform.GetElement(i, 3)) * scale_factor for i in range(3)]

    # divide position by scale factor
    displacementTransform.SetElement(
        0, 3, 0)
    displacementTransform.SetElement(
        1, 3, 0)
    displacementTransform.SetElement(
        2, 3, 0)

    return displacementTransform, positionDisplacementVector

if __name__ == "__main__":

    # Create a custom layout
    popw = createCustomLayout(resolution=[1280, 720])
    popw.show()  # only works if called in the main function
    stereo = StereoView()
    # add_model_to_scene("TumorModel.vtk")
    stereo.setup()



# exec(open('layout_custom.py').read())
