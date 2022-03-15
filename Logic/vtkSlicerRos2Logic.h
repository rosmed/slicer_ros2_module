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

// .NAME vtkSlicerRos2Logic - slicer logic class for volumes manipulation
// .SECTION Description
// This class manages the logic associated with reading, saving,
// and changing propertied of the volumes


#ifndef __vtkSlicerRos2Logic_h
#define __vtkSlicerRos2Logic_h

// Forward declarations
namespace KDL {
  class ChainFkSolverPos_recursive;
}

namespace rclcpp {
  class Node;
}

class vtkMRMLTransformNode;

// Slicer includes
#include <vtkSlicerModuleLogic.h>
#include <vtkSmartPointer.h>
#include "vtkSlicerRos2ModuleLogicExport.h"

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

/// \ingroup Slicer_QtModules_ExtensionTemplate
class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkSlicerRos2Logic :
  public vtkSlicerModuleLogic
{
public:

  static vtkSlicerRos2Logic *New();
  vtkTypeMacro(vtkSlicerRos2Logic, vtkSlicerModuleLogic);
  void PrintSelf(ostream& os, vtkIndent indent) override;
  void loadRobotSTLModels(const std::string & filename); // Could also be protected friend ** ask Anton
  void UpdateFK(const std::vector<double> & joinValues);
  void Spin(void);

protected:
  vtkSlicerRos2Logic();
  ~vtkSlicerRos2Logic() override;

  void SetMRMLSceneInternal(vtkMRMLScene* newScene) override;
  /// Register MRML Node classes to Scene. Gets called automatically when the MRMLScene is attached to this logic class.
  void RegisterNodes() override;
  void UpdateFromMRMLScene() override;
  void OnMRMLSceneNodeAdded(vtkMRMLNode* node) override;
  void OnMRMLSceneNodeRemoved(vtkMRMLNode* node) override;


private:

  vtkSlicerRos2Logic(const vtkSlicerRos2Logic&); // Not implemented
  void operator=(const vtkSlicerRos2Logic&); // Not implemented

  KDL::ChainFkSolverPos_recursive * mKDLSolver = 0;
  size_t mKDLChainSize = 0;
  std::vector<vtkSmartPointer<vtkMRMLTransformNode> > mChainNodeTransforms;

  void ParameterCallback(std::shared_future<std::vector<rclcpp::Parameter>> future);
  std::shared_ptr<rclcpp::Node> mNodePointer;
  std::shared_ptr<rclcpp::AsyncParametersClient> mParameterClient;

  std::string robot_description_string;
  bool parameterNodeCallbackFlag = false;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> mJointStateSubscription;
  void JointStateCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
};

#endif
