import slicer
import vtk
import numpy as np
from scipy.spatial.transform import Rotation as R
import qt

try:
    import yaml
except:
    pip_install('pyyaml')
    import yaml

def createCustomLayout(position, size):

    THREE_3D_LAYOUT = """
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
    layoutManager.layoutLogic().GetLayoutNode().AddLayoutDescription(customLayoutId, THREE_3D_LAYOUT)

    # Set the application layout to the custom layout
    layoutManager.setLayout(customLayoutId)

    # Extract the 3D widgets for stereo cameras
    threeDWidget1 = layoutManager.threeDWidget(0)
    threeDWidget2 = layoutManager.threeDWidget(1)

    # Create a new window for the stereo cameras
    popupWindow = qt.QWidget()
    popupWindow.setWindowTitle("Stereo Cameras")
    popupWindow.setLayout(qt.QHBoxLayout())
    popupWindow.layout().addWidget(threeDWidget1)
    popupWindow.layout().addWidget(threeDWidget2)
    popupWindow.show()

    popupWindow.move(100, 100)
    popupWindow.resize(1280*2, 720)

    return popupWindow


DEBUG_INTERPOLATION = 0.4 ##TODO clean this up

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