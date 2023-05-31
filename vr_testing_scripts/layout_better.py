import slicer
import vtk
import numpy as np
try:
    import yaml
except:
    pip_install('pyyaml')
    import yaml

rotation_correction_matrix = vtk.vtkMatrix4x4()

# Set the rotation matrix to be :
# 0 1 0 0
# 0 0 1 0
# 1 0 0 0
# 0 0 0 1

rotation_correction_matrix.SetElement(0, 0, 0)
rotation_correction_matrix.SetElement(0, 1, 1)
rotation_correction_matrix.SetElement(0, 2, 0)
rotation_correction_matrix.SetElement(0, 3, 0)

rotation_correction_matrix.SetElement(1, 0, 0)
rotation_correction_matrix.SetElement(1, 1, 0)
rotation_correction_matrix.SetElement(1, 2, 1)
rotation_correction_matrix.SetElement(1, 3, 0)

rotation_correction_matrix.SetElement(2, 0, 1)
rotation_correction_matrix.SetElement(2, 1, 0)
rotation_correction_matrix.SetElement(2, 2, 0)
rotation_correction_matrix.SetElement(2, 3, 0)

rotation_correction_matrix.SetElement(3, 0, 0)
rotation_correction_matrix.SetElement(3, 1, 0)
rotation_correction_matrix.SetElement(3, 2, 0)
rotation_correction_matrix.SetElement(3, 3, 1)


class StereoView:
    def __init__(self):
        self.layout_manager = slicer.app.layoutManager()
        self.threeDWidget1 = self.layout_manager.threeDWidget(0)
        self.threeDWidget2 = self.layout_manager.threeDWidget(1)
        # Get the view nodes from the 3D widgets
        self.viewNode1 = self.threeDWidget1.mrmlViewNode()
        self.viewNode2 = self.threeDWidget2.mrmlViewNode()
        # Get the active camera nodes for both views
        self.camera_node1 = slicer.modules.cameras.logic(
        ).GetViewActiveCameraNode(self.viewNode1)
        self.camera_node2 = slicer.modules.cameras.logic(
        ).GetViewActiveCameraNode(self.viewNode2)

        self.transformation_to_center = vtk.vtkMatrix4x4()

    def set_separation(self, x, y, z, reset=True):  # Unrefined

        self.center_to_left = vtk.vtkMatrix4x4()
        self.center_to_right = vtk.vtkMatrix4x4()

        self.center_to_left.SetElement(0, 3, -x / 2)
        self.center_to_right.SetElement(0, 3, x / 2)

        self.center_to_left.SetElement(1, 3, -y / 2)
        self.center_to_right.SetElement(1, 3, y / 2)

        self.center_to_left.SetElement(2, 3, -z / 2)
        self.center_to_right.SetElement(2, 3, z / 2)

    def displace_camera(self, displacement_transform, translation_vector):

        self.left_camera_transform_matrix = vtk.vtkMatrix4x4()
        self.right_camera_transform_matrix = vtk.vtkMatrix4x4()
        new_center_transform_matrix = vtk.vtkMatrix4x4()

        # multiply just the rotation part of the displacement transform with the initial center transform matrix
        initial_center_position = [self.initial_center_transform_matrix.GetElement(
            0, 3), self.initial_center_transform_matrix.GetElement(1, 3), self.initial_center_transform_matrix.GetElement(2, 3)]

        # set the translation part of the initial center transform matrix to 0
        self.initial_center_transform_matrix.SetElement(0, 3, 0)
        self.initial_center_transform_matrix.SetElement(1, 3, 0)
        self.initial_center_transform_matrix.SetElement(2, 3, 0)

        vtk.vtkMatrix4x4.Multiply4x4(
            displacement_transform, self.initial_center_transform_matrix, new_center_transform_matrix)

        # Add translation vector to the new center transform matrix
        new_center_transform_matrix.SetElement(
            0, 3,  translation_vector[0] + initial_center_position[0])
        new_center_transform_matrix.SetElement(
            1, 3,  translation_vector[1] + initial_center_position[1])
        new_center_transform_matrix.SetElement(
            2, 3,  translation_vector[2] + initial_center_position[2])

        self.center_transform_matrix_end_state = new_center_transform_matrix

        vtk.vtkMatrix4x4.Multiply4x4(
            new_center_transform_matrix, self.center_to_left, self.left_camera_transform_matrix)
        vtk.vtkMatrix4x4.Multiply4x4(
            new_center_transform_matrix, self.center_to_right, self.right_camera_transform_matrix)

        self.left_camera_transform.SetMatrixTransformToParent(
            self.left_camera_transform_matrix)
        self.camera_node1.SetNodeReferenceID(
            "transform", self.left_camera_transform.GetID())

        self.right_camera_transform.SetMatrixTransformToParent(
            self.right_camera_transform_matrix)
        self.camera_node2.SetNodeReferenceID(
            "transform", self.right_camera_transform.GetID())

        self.initial_center_transform_matrix.DeepCopy(
            new_center_transform_matrix)

        print("Displaced!")

    def setup(self):
        ros2Node = slicer.mrmlScene.GetFirstNodeByName('ros2:node:slicer')
        self.lookupNode = ros2Node.CreateAndAddTf2LookupNode(
            "MTMR_base", "MTMR")
        self.lookupNodeID = self.lookupNode.GetID()
        self.scale_factor = 5

        self.buttonSubscriber = ros2Node.CreateAndAddSubscriberNode(
            "vtkMRMLROS2SubscriberJoyNode", "/console/camera")

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

        translate = [0, 0, 0]

        for i in range(2):
            # FIXME: Unsure of why this is necessary
            self.displace_camera(displacement, translate)
            self.displace_camera(rotation_correction_matrix, translate)

        self.initial_center_transform_matrix.DeepCopy(
            self.center_transform_matrix_end_state)

        self.buttonObserverID = self.buttonSubscriber.AddObserver(
            "ModifiedEvent", self._buttonCallback)
        observerId = self.lookupNode.AddObserver(
            slicer.vtkMRMLTransformNode.TransformModifiedEvent, self._callback)

    def _buttonCallback(self, caller, event):
        msg_yaml = caller.GetLastMessageYAML()
        msg = yaml.load(msg_yaml, Loader=yaml.FullLoader)
        button = msg['buttons'][0]

        if button == 1 and self.move_robot == False:
            self.start_motion()
        elif button == 0 and self.move_robot == True:
            self.stop_motion()
        else:
            print(f"Received button value: {button}")

    def start_motion(self):
        self.initial_robot_transform = vtk.vtkMatrix4x4()
        self.initial_robot_transform.DeepCopy(
            self.lookupNode.GetMatrixTransformToParent())
        self.move_robot = True
        print("start transform:")
        for i in range(4):
            print(f"{self.initial_robot_transform.GetElement(i, 0):+0.1f} {self.initial_robot_transform.GetElement(i, 1):+0.1f} {self.initial_robot_transform.GetElement(i, 2):+0.1f} {self.initial_robot_transform.GetElement(i, 3):+0.1f}")

    def stop_motion(self):
        self.move_robot = False
        self.initial_center_transform_matrix = self.center_transform_matrix_end_state
        print("end transform:")
        for i in range(4):
            print(f"{self.lastTransform.GetElement(i, 0):+0.1f} {self.lastTransform.GetElement(i, 1):+0.1f} {self.lastTransform.GetElement(i, 2):+0.1f} {self.lastTransform.GetElement(i, 3):+0.1f}")

    def _callback(self, caller, event):
        if self.move_robot:
            self.lastTransform = caller.GetMatrixTransformToParent()
            displacement, translation_vector = findDisplacementTransform(
                self.initial_robot_transform, self.lastTransform, self.scale_factor)
            self.displace_camera(displacement, translation_vector)
            # copy contents of lastTransform into initial_robot_transform they are vtkMatrix4x4 objects
            self.initial_robot_transform.DeepCopy(self.lastTransform)

    def calibrate(self):
        self.camera_node1.SetViewUp([0, 0, 1])
        self.camera_node2.SetViewUp([0, 0, 1])

        # self.camera_node1.SetFocalPoint([0, 0, 0])
        # self.camera_node2.SetFocalPoint([0, 0, 0])

        # self.camera_node1.SetPosition([0, 0, 0])
        # self.camera_node2.SetPosition([0, 0, 0])


def add_model_to_scene(path_to_model):
    model = slicer.util.loadModel(path_to_model)
    if not model:
        print("Error loading model")
        return


def create_custom_layout(position=[100, 100], resolution=[1920, 1080]):
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

    translation_vector = [(endTransform.GetElement(
        i, 3) - startTransform.GetElement(i, 3)) * scale_factor for i in range(3)]

    # divide position by scale factor
    displacementTransform.SetElement(
        0, 3, 0)
    displacementTransform.SetElement(
        1, 3, 0)
    displacementTransform.SetElement(
        2, 3, 0)

    return displacementTransform, translation_vector


if __name__ == "__main__":

    # Create a custom layout
    popw = create_custom_layout(resolution=[1280, 720])
    popw.show()  # only works if called in the main function
    stereo = StereoView()
    # add_model_to_scene("TumorModel.vtk")
    stereo.calibrate()
    # add_model_to_scene("base.stl")
    stereo.set_separation(0, 0, 0)

    stereo.setup()
    # stereo.start_motion()


# exec(open('layout_better.py').read())
