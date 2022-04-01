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

// Tf include_directories
// Tf includes
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <turtlesim/srv/spawn.hpp>

#include <chrono>
#include <memory>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;


/// \ingroup Slicer_QtModules_ExtensionTemplate
class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkSlicerRos2Logic :
  public vtkSlicerModuleLogic
{
public:

  static vtkSlicerRos2Logic *New();
  vtkTypeMacro(vtkSlicerRos2Logic, vtkSlicerModuleLogic);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  // Logic methods

  /*! This method will set the name of the node and name of the
    parameter used to load the robot URDF.  Once the names are set,
    the class will create a parameter client and callback to retrieve
    the URDF. */
  void SetModelNodeAndParameter(const std::string & nodeName,
				const std::string & parameterName);
  void SetModelFile(const std::string & selectedFile);
  void SetRobotStateTopic(const std::string & topicName);
  void SetRobotStateTf();
  void loadRobotSTLModels(); // Could also be protected friend ** ask Anton
  void UpdateFK(const std::vector<double> & joinValues);
  void Spin(void);
  void Clear();

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

  void ModelParameterCallback(std::shared_future<std::vector<rclcpp::Parameter>> future);
  std::shared_ptr<rclcpp::Node> mNodePointer;
  std::shared_ptr<rclcpp::AsyncParametersClient> mParameterClient;

  bool parameterNodeCallbackFlag = false;
  std::vector<std::string> link_names_vector;
  std::vector<std::string> link_parent_names_vector;

  // state
  struct {
    bool IsUsingTopic = false; // if not topic, using tf
    std::string Topic;
  } mRobotState;

  struct {
    bool Loaded = false; // do we have a model properly loaded and something to display
    std::string URDF; // keep a copy of the URDF before it's loaded
    bool ComesFromFile = false; // by default we assume the URDF comes for parameter
    std::string FileName;
    bool Serial = true;
    struct {
      std::string NodeName;
      std::string ParameterName;
      bool NodeFound = false;
      bool ParameterFound = false;
    } Parameter;
  } mModel;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> mJointStateSubscription;
  void JointStateCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);

  std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> mTfListener;
  void queryTfNode();
  void updateTransformFromTf(geometry_msgs::msg::TransformStamped transformStamped, int transformCount);


};

#endif
