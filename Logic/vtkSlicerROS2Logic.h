/*==============================================================================

  Program: 3D Slicer

  Portions (c) Copyright Brigham and Women's Hospital (BWH) All Rights Reserved.

  See COPYRIGHT.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

==============================================================================*/

// .NAME vtkSlicerROS2Logic - slicer logic class for volumes manipulation
// .SECTION Description
// This class manages the logic associated with reading, saving,
// and changing propertied of the volumes


#ifndef __vtkSlicerROS2Logic_h
#define __vtkSlicerROS2Logic_h

// foward declarations
// VTK
class vtkTimerLog;

// Slicer
class vtkMRMLROS2NodeNode;
class vtkMRMLROS2SubscriberNode;
class vtkMRMLROS2PublisherNode;
class vtkMRMLROS2ParameterNode;
class vtkMRMLROS2Tf2BroadcasterNode;
class vtkMRMLROS2RobotNode;

// Slicer includes
#include <vtkSlicerModuleLogic.h>
#include <vtkSmartPointer.h>
#include <vtkSlicerROS2ModuleLogicExport.h>

// REMOVE THIS LATER
#include <vtkROS2SensorMsgsJoy.h>

/// \ingroup Slicer_QtModules_ExtensionTemplate
class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkSlicerROS2Logic:
  public vtkSlicerModuleLogic
{
 public:
  static vtkSlicerROS2Logic *New();
  vtkTypeMacro(vtkSlicerROS2Logic, vtkSlicerModuleLogic);
  void PrintSelf(ostream& os, vtkIndent indent) override;

 protected:
  vtkSlicerROS2Logic();
  ~vtkSlicerROS2Logic() override;

  void SetMRMLSceneInternal(vtkMRMLScene* newScene) override;
  /// Register MRML Node classes to Scene. Gets called automatically when the MRMLScene is attached to this logic class.
  void RegisterNodes() override;
  void UpdateFromMRMLScene() override;
  void OnMRMLSceneNodeAdded(vtkMRMLNode* node) override;
  void OnMRMLSceneNodeRemoved(vtkMRMLNode* node) override;

 public:
  // REMOVE THIS LATER
  inline vtkSmartPointer<vtkROS2SensorMsgsJoy> GetSensor(void) {
    vtkNew<vtkROS2SensorMsgsJoy> sensor;
    return sensor;
  }
   
  /*! Spin all the ROS nodes attached to the module's core logic.
    This method needs to be called periodically to dequeue all the ROS
    incomming messages (subscriptions, parameters and tf2 lookups.  By
    default, this method is called using a Qt timer that will run as
    soon as this modules logic widget is displayed.  The default
    frequency is 50Hz. */
  void Spin(void);

  /*! Get the default ROS node attached to the core logic.  The
    default ROS node can be used for most applications.  It is started
    without any namespace.  If your application requires a ROS
    namespace, create a new ROS node (vtkMRMLROS2NodeNode) and add it
    to the MRML scene.  The core logic will automatically detect any
    new ROS node added to the scene and make sure the node is
    "spinning". */
  vtkMRMLROS2NodeNode * GetDefaultROS2Node(void) const;

  void AddRobot(const std::string & robotName, const std::string & parameterNodeName, const std::string & parameterName);
  void RemoveRobot(const std::string & robotName);

  vtkSmartPointer<vtkMRMLROS2NodeNode> mDefaultROS2Node; // should this be private?? UI needs to access it

 private:

  vtkSlicerROS2Logic(const vtkSlicerROS2Logic&); // Not implemented
  void operator=(const vtkSlicerROS2Logic&); // Not implemented


  std::vector<vtkSmartPointer<vtkMRMLROS2NodeNode> > mROS2Nodes;
  vtkSmartPointer<vtkTimerLog> mTimerLog;
};

#endif
