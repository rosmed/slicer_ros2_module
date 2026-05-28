import slicer
import vtk
import qt

# 1. Build sphere geometry and a transform filter to animate it
sphere = vtk.vtkSphereSource()
sphere.SetRadius(15.0)
sphere.SetThetaResolution(20)
sphere.SetPhiResolution(20)
sphere.Update()

transform = vtk.vtkTransform()
transformFilter = vtk.vtkTransformPolyDataFilter()
transformFilter.SetInputConnection(sphere.GetOutputPort())
transformFilter.SetTransform(transform)
transformFilter.Update()

# 2. Create a Model node seeded with the initial geometry
modelNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'AnimatedPointCloud')
modelNode.SetAndObservePolyData(vtk.vtkPolyData())
modelNode.GetPolyData().DeepCopy(transformFilter.GetOutput())
modelNode.CreateDefaultDisplayNodes()

# 3. Set up the ROS 2 publisher
rosLogic = slicer.util.getModuleLogic('ROS2')
rosNode = rosLogic.GetDefaultROS2Node()
pub = rosNode.CreateAndAddPublisherNode('PolyData', '/slicer_point_cloud')
pub.SetFrameId('world')

# 4. Auto-republish whenever the model node is marked modified.
#    This also fires if the user edits the model interactively in Slicer.
def onModelModified(caller, event):
    pub.Publish(caller.GetPolyData())

modelNode.AddObserver(vtk.vtkCommand.ModifiedEvent, onModelModified)

# Publish the initial frame immediately
pub.Publish(modelNode.GetPolyData())

# 5. Drive the animation at 10 Hz: rotate the sphere 5 degrees per step
angle = [0.0]

def animate():
    angle[0] = (angle[0] + 5.0) % 360.0
    transform.Identity()
    transform.RotateZ(angle[0])
    transformFilter.Update()
    modelNode.GetPolyData().DeepCopy(transformFilter.GetOutput())
    modelNode.Modified()  # triggers onModelModified -> pub.Publish()

timer = qt.QTimer()
timer.setInterval(100)  # 10 Hz
timer.connect('timeout()', animate)
timer.start()

# Keep a reference so the timer is not garbage-collected
slicer.modules.AnimatedPointCloudTimer = timer

print("Animated point cloud publisher started at 10 Hz on /slicer_point_cloud.")
print("The model will also republish automatically if you edit it in Slicer.")
