import slicer
import vtk
import numpy as np

class StereoView:
    def __init__(self):
        self.layout_manager = slicer.app.layoutManager()
        self.threeDWidget1 = self.layout_manager.threeDWidget(0)
        self.threeDWidget2 = self.layout_manager.threeDWidget(1)
        # Get the view nodes from the 3D widgets
        self.viewNode1 = self.threeDWidget1.mrmlViewNode()
        self.viewNode2 = self.threeDWidget2.mrmlViewNode()
        # Get the active camera nodes for both views
        self.camera_node1 = slicer.modules.cameras.logic().GetViewActiveCameraNode(self.viewNode1)
        self.camera_node2 = slicer.modules.cameras.logic().GetViewActiveCameraNode(self.viewNode2)

        self.transformation_to_center = vtk.vtkMatrix4x4()

    def set_separation(self, x, y, z, reset=True): # Unrefined

        self.center_to_left = vtk.vtkMatrix4x4()
        self.center_to_right = vtk.vtkMatrix4x4()

        self.center_to_left.SetElement(0, 3, -x / 2)
        self.center_to_right.SetElement(0, 3, x / 2)

        self.center_to_left.SetElement(1, 3, -y / 2)
        self.center_to_right.SetElement(1, 3, y / 2)

        self.center_to_left.SetElement(2, 3, -z / 2)
        self.center_to_right.SetElement(2, 3, z / 2)


    def displace_camera(self, displacement_transform):

        self.left_camera_transform_matrix = vtk.vtkMatrix4x4()
        self.right_camera_transform_matrix = vtk.vtkMatrix4x4()
        new_center_transform_matrix = vtk.vtkMatrix4x4()

        vtk.vtkMatrix4x4.Multiply4x4(displacement_transform, self.initial_center_transform_matrix, new_center_transform_matrix)
        # vtk.vtkMatrix4x4.Multiply4x4(displacement_transform, self.initial_right_camera_transform, self.right_camera_transform_matrix)
        self.center_transform_matrix_end_state = new_center_transform_matrix

        vtk.vtkMatrix4x4.Multiply4x4(new_center_transform_matrix, self.center_to_left, self.left_camera_transform_matrix)
        vtk.vtkMatrix4x4.Multiply4x4(new_center_transform_matrix, self.center_to_right, self.right_camera_transform_matrix)

        print("New center transform matrix: ")
        print(new_center_transform_matrix)

        print("New left camera transform matrix: ")
        print(self.left_camera_transform_matrix)

        print("New right camera transform matrix: ")
        print(self.right_camera_transform_matrix)

        self.left_camera_transform.SetMatrixTransformToParent(self.left_camera_transform_matrix)
        self.camera_node1.SetNodeReferenceID("transform", self.left_camera_transform.GetID())

        self.right_camera_transform.SetMatrixTransformToParent(self.right_camera_transform_matrix)
        self.camera_node2.SetNodeReferenceID("transform", self.right_camera_transform.GetID())

        print("Displaced!")


    def setup(self):
        ros2Node = slicer.mrmlScene.GetFirstNodeByName('ros2:node:slicer')
        self.lookupNode = ros2Node.CreateAndAddTf2LookupNode("MTMR_base","MTMR")
        self.lookupNodeID = self.lookupNode.GetID()
        self.scale_factor = 10

        if not slicer.mrmlScene.GetFirstNodeByName("left_camera_transform"):
            self.left_camera_transform = slicer.vtkMRMLLinearTransformNode()
            self.left_camera_transform.SetName("left_camera_transform")
            slicer.mrmlScene.AddNode(self.left_camera_transform)

        if not slicer.mrmlScene.GetFirstNodeByName("right_camera_transform"):
            self.right_camera_transform = slicer.vtkMRMLLinearTransformNode()
            self.right_camera_transform.SetName("right_camera_transform")
            slicer.mrmlScene.AddNode(self.right_camera_transform)

        self.initial_center_transform_matrix = vtk.vtkMatrix4x4()

        displacement = vtk.vtkMatrix4x4()

        self.move_robot = False

        for i in range(2):
            self.displace_camera(displacement) # FIXME: Unsure of why this is necessary

        observerId = self.lookupNode.AddObserver(slicer.vtkMRMLTransformNode.TransformModifiedEvent, self._callback)

    def start_motion(self):
        self.initial_robot_transform = vtk.vtkMatrix4x4() 
        self.initial_robot_transform.DeepCopy(self.lookupNode.GetMatrixTransformToParent())
        self.move_robot = True

    def stop_motion(self):
        self.move_robot = False
        self.center_transform_matrix = self.center_transform_matrix_end_state

    def _callback(self, caller, event):
        if self.move_robot:
            self.lastTransform = caller.GetMatrixTransformToParent()
            displacement = findDisplacementTransform(self.initial_robot_transform, self.lastTransform, self.scale_factor)
            self.displace_camera(displacement)
        

def add_model_to_scene(path_to_model):
    model = slicer.util.loadModel(path_to_model)
    if not model:
        print("Error loading model")
        return
        
def create_custom_layout(position = [100, 100], resolution = [1920, 1080]):
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
    customLayoutId=100 # an arbitrary ID number for the custom layout
    layoutManager.layoutLogic().GetLayoutNode().AddLayoutDescription(customLayoutId, customLayout)
    # Set the application layout to the custom layout
    layoutManager.setLayout(customLayoutId)

    threeDWidget1 = layoutManager.threeDWidget(0)
    threeDWidget2 = layoutManager.threeDWidget(1)

    popupWindow = qt.QWidget()
    popupWindow.setWindowTitle("Test ABCD")
    popupWindow.setLayout(qt.QHBoxLayout())
    popupWindow.layout().addWidget(threeDWidget1)
    popupWindow.layout().addWidget(threeDWidget2)

    popupWindow.move(*position)  # Number of pixels from the left and top of the screen
    popupWindow.resize(*resolution)

    return popupWindow


def findDisplacementTransform(startTransform, endTransform, scale_factor):
    displacementTransform = vtk.vtkMatrix4x4()

    # end = displacement * start
    # displacement = end * start^-1
    startTransformInverse = vtk.vtkMatrix4x4()
    vtk.vtkMatrix4x4.Invert(startTransform, startTransformInverse)
    vtk.vtkMatrix4x4.Multiply4x4(endTransform, startTransformInverse, displacementTransform)
    # vtk.vtkMatrix4x4.Multiply4x4(startTransformInverse, endTransform, displacementTransform)

    # divide position by scale factor
    displacementTransform.SetElement(0, 3, displacementTransform.GetElement(0, 3) / scale_factor)
    displacementTransform.SetElement(1, 3, displacementTransform.GetElement(1, 3) / scale_factor)
    displacementTransform.SetElement(2, 3, displacementTransform.GetElement(2, 3) / scale_factor)

    return displacementTransform

if __name__ == "__main__":

    # Create a custom layout
    # popw = create_custom_layout(resolution=[1280, 720])
    popw = create_custom_layout(resolution=[1920, 1080])
    popw.show() # only works if called in the main function
    stereo = StereoView()
    add_model_to_scene("TumorModel.vtk")
    # add_model_to_scene("base.stl")
    stereo.set_separation(10, 10, 10)

    stereo.setup()
    # stereo.start_motion()
    



    


# exec(open('layout_better.py').read()) 