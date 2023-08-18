import slicer
import vtk
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
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


def findDisplacementTransform(startTransform, endTransform, translationScaleFactor = 0.3, rotationScaleFactor = 0.4):
    displacementTransform = vtk.vtkMatrix4x4()

    # end = displacement * start
    # displacement = end * start^-1
    startTransformInverse = vtk.vtkMatrix4x4()
    vtk.vtkMatrix4x4.Invert(startTransform, startTransformInverse)
    vtk.vtkMatrix4x4.Multiply4x4(
        endTransform, startTransformInverse, displacementTransform)

    positionDisplacementVector = [(endTransform.GetElement(
        i, 3) - startTransform.GetElement(i, 3)) * translationScaleFactor for i in range(3)]

    # divide position by scale factor
    displacementTransform.SetElement(
        0, 3, 0)
    displacementTransform.SetElement(
        1, 3, 0)
    displacementTransform.SetElement(
        2, 3, 0)
    
    # displacementTransform = slerp_vtk_rotation_matrix(displacementTransform)
    displacementTransform = interpolateVTKMatrix(displacementTransform, rotationScaleFactor)

    return displacementTransform, positionDisplacementVector

def slerpScipy(r0, r1, t):
    # Times sequence for the start and end rotations
    times = [0, 1]
    combinedRotations = R.concatenate([r0, r1])
    # Create the Slerp object
    slerpInterpolator = Slerp(times, combinedRotations)
    # Obtain the interpolated rotation
    slerpRotation = slerpInterpolator([t])[0]

    # Return the quaternion of the interpolated rotation
    return slerpRotation

def interpolateVTKMatrix(vtkMatrix, t):
    # Convert vtkMatrix4x4 to numpy array
    matrixArray = np.array([[vtkMatrix.GetElement(i, j) for j in range(4)] for i in range(4)])

    # Extract 3x3 rotation matrix
    rMat = matrixArray[0:3, 0:3]
    r1 = R.from_matrix(rMat)

    # Perform slerp between identity quaternion and rotation
    r0 = R.from_euler('xyz', [0, 0, 0])  # Create an identity rotation

    rScaled = slerpScipy(r0, r1, t)
    rTransformed = rScaled.as_matrix()

    # Construct the resulting vtkMatrix4x4
    resultVTKMatrix = vtk.vtkMatrix4x4()
    for i in range(3):
        for j in range(3):
            resultVTKMatrix.SetElement(i, j, rTransformed[i, j])

    return resultVTKMatrix
