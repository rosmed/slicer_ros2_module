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

    def set_separation(self, x, y, z, reset=True):

        self.center_to_left = vtk.vtkMatrix4x4()
        self.center_to_right = vtk.vtkMatrix4x4()

        self.center_to_left.SetElement(0, 3, -x / 2)
        self.center_to_right.SetElement(0, 3, x / 2)

        self.center_to_left.SetElement(1, 3, -y / 2)
        self.center_to_right.SetElement(1, 3, y / 2)

        self.center_to_left.SetElement(2, 3, -z / 2)
        self.center_to_right.SetElement(2, 3, z / 2)

        left_transformed_matrix = vtk.vtkMatrix4x4()
        right_transformed_matrix = vtk.vtkMatrix4x4()

        vtk.vtkMatrix4x4.Multiply4x4(self.transformation_to_center, self.center_to_left, left_transformed_matrix)
        vtk.vtkMatrix4x4.Multiply4x4(self.transformation_to_center, self.center_to_right, right_transformed_matrix)

        new_position1 = [left_transformed_matrix.GetElement(0, 3), left_transformed_matrix.GetElement(1, 3), left_transformed_matrix.GetElement(2, 3)]
        new_position2 = [right_transformed_matrix.GetElement(0, 3), right_transformed_matrix.GetElement(1, 3), right_transformed_matrix.GetElement(2, 3)]

        self.camera_node1.SetPosition(new_position1)
        self.camera_node2.SetPosition(new_position2)

        self.camera_node1.SetViewUp(left_transformed_matrix.GetElement(0, 2), left_transformed_matrix.GetElement(1, 2), left_transformed_matrix.GetElement(2, 2))
        self.camera_node2.SetViewUp(right_transformed_matrix.GetElement(0, 2), right_transformed_matrix.GetElement(1, 2), right_transformed_matrix.GetElement(2, 2))

        print("Separated!")

    def move_camera_position(self, new_transform_matrix):
        self.transformation_to_center = new_transform_matrix

        left_transformed_matrix = vtk.vtkMatrix4x4()
        right_transformed_matrix = vtk.vtkMatrix4x4()

        vtk.vtkMatrix4x4.Multiply4x4(self.transformation_to_center, self.center_to_left, left_transformed_matrix)
        vtk.vtkMatrix4x4.Multiply4x4(self.transformation_to_center, self.center_to_right, right_transformed_matrix)

        self.camera_node1.SetPosition(left_transformed_matrix.GetElement(0, 3), left_transformed_matrix.GetElement(1, 3), left_transformed_matrix.GetElement(2, 3))
        self.camera_node2.SetPosition(right_transformed_matrix.GetElement(0, 3), right_transformed_matrix.GetElement(1, 3), right_transformed_matrix.GetElement(2, 3))

        self.camera_node1.SetViewUp(left_transformed_matrix.GetElement(0, 2), left_transformed_matrix.GetElement(1, 2), left_transformed_matrix.GetElement(2, 2))
        self.camera_node2.SetViewUp(right_transformed_matrix.GetElement(0, 2), right_transformed_matrix.GetElement(1, 2), right_transformed_matrix.GetElement(2, 2))

        self.camera_node1.SetFocalPoint(left_transformed_matrix.GetElement(0, 1), left_transformed_matrix.GetElement(1, 1), left_transformed_matrix.GetElement(2, 1))
        self.camera_node2.SetFocalPoint(right_transformed_matrix.GetElement(0, 1), right_transformed_matrix.GetElement(1, 1), right_transformed_matrix.GetElement(2, 1))


        print("Moved!")
        

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

def create_vtk_transform_matrix(alpha, beta, gamma, x, y, z):
    # Convert angles from degrees to radians
    alpha = np.radians(alpha)
    beta = np.radians(beta)
    gamma = np.radians(gamma)

    # Rotation matrices for each axis
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(alpha), -np.sin(alpha)],
                   [0, np.sin(alpha), np.cos(alpha)]])

    Ry = np.array([[np.cos(beta), 0, np.sin(beta)],
                   [0, 1, 0],
                   [-np.sin(beta), 0, np.cos(beta)]])

    Rz = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                   [np.sin(gamma), np.cos(gamma), 0],
                   [0, 0, 1]])

    # Combine rotations
    R = Rz @ Ry @ Rx

    # Create a vtkMatrix4x4 instance
    vtk_matrix = vtk.vtkMatrix4x4()

    # Set the elements of the vtkMatrix4x4 (row, column, value)
    for i in range(3):
        for j in range(3):
            vtk_matrix.SetElement(i, j, R[i, j])

    # Set the translation elements
    vtk_matrix.SetElement(0, 3, x)
    vtk_matrix.SetElement(1, 3, y)
    vtk_matrix.SetElement(2, 3, z)

    # Set the homogeneous coordinate elements
    vtk_matrix.SetElement(3, 0, 0)
    vtk_matrix.SetElement(3, 1, 0)
    vtk_matrix.SetElement(3, 2, 0)
    vtk_matrix.SetElement(3, 3, 1)

    return vtk_matrix

def helper_create_and_move(alpha, beta, gamma, x, y, z):
    transform_matrix_vtk = create_vtk_transform_matrix(alpha, beta, gamma, x, y, z)
    stereo.move_camera_position(transform_matrix_vtk)

class TestObserverTf2Lookup:
    def __init__(self):
        self.counter = 0

    def Callback(self, caller, event):
        self.counter += 1
        self.lastTransform = caller.GetMatrixTransformToParent()


def findDisplacementTransform(startTransform, endTransform):
    displacementTransform = vtk.vtkMatrix4x4()
    # start * displacement = end
    # displacement = start^-1 * end
    startTransformInverse = vtk.vtkMatrix4x4()
    vtk.vtkMatrix4x4.Invert(startTransform, startTransformInverse)

    vtk.vtkMatrix4x4.Multiply4x4(startTransformInverse, endTransform, displacementTransform)

    return displacementTransform

# write a unit test for this
def testFindDisplacementTransform():
    startMatrix = vtk.vtkMatrix4x4()
    startMatrix.SetElement(0, 3, 1)
    startMatrix.SetElement(1, 3, 2)
    startMatrix.SetElement(2, 3, 3)

    endMatrix = vtk.vtkMatrix4x4()
    endMatrix.SetElement(0, 3, 4)
    endMatrix.SetElement(1, 3, 5)
    endMatrix.SetElement(2, 3, 6)


    displacementMatrix4x4= findDisplacementTransform(startMatrix, endMatrix)
    
    assert(displacementMatrix4x4.GetElement(0, 3) == 3)
    assert(displacementMatrix4x4.GetElement(1, 3) == 3)
    assert(displacementMatrix4x4.GetElement(2, 3) == 3)

    assert(displacementMatrix4x4.GetElement(0, 0) == 1)
    assert(displacementMatrix4x4.GetElement(1, 1) == 1)
    assert(displacementMatrix4x4.GetElement(2, 2) == 1)

    print("Test passed")


if __name__ == "__main__":

    print("Testing displacement calculation")
    testFindDisplacementTransform()
    # Create a custom layout

    popw = create_custom_layout(resolution=[1280, 720])
    popw.show() # only works if called in the main function
    stereo = StereoView()
    # add_model_to_scene("TumorModel.vtk")
    add_model_to_scene("base.stl")
    stereo.set_separation(0, 0, 0)

    # # Example usage:
    # alpha = 30  # rotation around X-axis in degrees
    # beta = 45   # rotation around Y-axis in degrees
    # gamma = 60  # rotation around Z-axis in degrees
    # x, y, z = 0.35, 1.26, 0.289

    # helper_create_and_move(alpha, beta, gamma, x, y, z)

    ros2Node = slicer.mrmlScene.GetFirstNodeByName('ros2:node:slicer')
    lookupNode = ros2Node.CreateAndAddTf2LookupNode("MTMR_base","MTMR")

    observer = TestObserverTf2Lookup()
    observerId = lookupNode.AddObserver(slicer.vtkMRMLTransformNode.TransformModifiedEvent, observer.Callback)
    lookupMat = observer.lastTransform

    # When lookupMat is updated, we need to update the position of the model
    if False:
        curr_count = observer.counter
        while True:
            if observer.count > curr_count:
                curr_count = observer.count
                displacementTransform = findDisplacementTransform(lookupMat, observer.lastTransform)

    


# exec(open('layout_better.py').read()) 