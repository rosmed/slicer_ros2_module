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

class vtkMRMLROS2NodeNode;
class vtkMRMLROS2SubscriberNode;
class vtkMRMLROS2PublisherNode;
class vtkMRMLROS2ParameterNode;
class vtkMRMLROS2Tf2BroadcasterNode;
class vtkMRMLROS2RobotNode;

// Slicer includes
#include <vtkSlicerModuleLogic.h>
#include <vtkSmartPointer.h>
#include "vtkSlicerROS2ModuleLogicExport.h"

#include "vtkMRML.h"
#include "vtkMRMLNode.h"

#include <vtkMatrix4x4.h>



/// \ingroup Slicer_QtModules_ExtensionTemplate
class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkSlicerROS2Logic:
  public vtkSlicerModuleLogic
{
public:

  static vtkSlicerROS2Logic *New();
  vtkTypeMacro(vtkSlicerROS2Logic, vtkSlicerModuleLogic);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  // Logic methods
  void Spin(void);

  /*! Developement methods, to be removed */
  vtkSmartPointer<vtkMRMLROS2NodeNode> mTestROS2Node;
  void AddROS2Node(void);
  void AddSomePublishers(void);
  void AddSomeSubscribers(void);
  void AddSomeParameters(void);
  void AddSomeTf2Nodes(void);
  void AddRobot(void);

protected:
  vtkSlicerROS2Logic();
  ~vtkSlicerROS2Logic() override;

  void SetMRMLSceneInternal(vtkMRMLScene* newScene) override;
  /// Register MRML Node classes to Scene. Gets called automatically when the MRMLScene is attached to this logic class.
  void RegisterNodes() override;
  void UpdateFromMRMLScene() override;
  void OnMRMLSceneNodeAdded(vtkMRMLNode* node) override;
  void OnMRMLSceneNodeRemoved(vtkMRMLNode* node) override;

private:

  vtkSlicerROS2Logic(const vtkSlicerROS2Logic&); // Not implemented
  void operator=(const vtkSlicerROS2Logic&); // Not implemented

  std::vector<vtkMRMLROS2NodeNode*> mROS2Nodes;
};

#endif
