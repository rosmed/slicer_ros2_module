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

// Forward declarations
namespace KDL {
  class ChainFkSolverPos_recursive;
}

namespace rclcpp {
  class Node;
}

class vtkMRMLTransformNode;
class vtkMRMLROS2SubscriberNode;
class vtkMRMLROS2PublisherNode;
class vtkMRMLROS2ParameterNode;
class vtkMRMLROS2Tf2BroadcasterNode;

// Slicer includes
#include <vtkSlicerModuleLogic.h>
#include <vtkSmartPointer.h>
#include "vtkSlicerROS2ModuleLogicExport.h"

#include "vtkMRML.h"
#include "vtkMRMLNode.h"

#include <vtkMatrix4x4.h>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

class vtkMRMLROS2NODENode;

/// \ingroup Slicer_QtModules_ExtensionTemplate
class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkSlicerROS2Logic:
  public vtkSlicerModuleLogic
{
public:

  static vtkSlicerROS2Logic *New();
  vtkTypeMacro(vtkSlicerROS2Logic, vtkSlicerModuleLogic);
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
  void BroadcastTransform();

  /*! Helper method to create a subscriber given a subscriber type and
    a topic. This method will create the corresponding MRML node if
    there is no existing subscriber for the given topic and add it to
    the default ROS2 node for this logic. It will return a nullptr a
    new subscriber was not created. */
  vtkMRMLROS2SubscriberNode * CreateAndAddSubscriber(const char * className, const std::string & topic);

  /*! Helper method to create a publisher given a publisher type and
    a topic. This method will create the corresponding MRML node if
    there is no existing publisher for the given topic and add it to
    the default ROS2 node for this logic. It will return a nullptr a
    new publisher was not created. */
  vtkMRMLROS2PublisherNode * CreateAndAddPublisher(const char * className, const std::string & topic);

  vtkMRMLROS2ParameterNode * CreateAndAddParameter(const char * className, const std::string & topic);

  void AddROS2Node(void);
  void AddSomePublishers(void);
  void AddSomeSubscribers(void);
  void AddSomeParameters(void);

  // std::vector<vtkSmartPointer<vtkMRMLROS2SubscriberNode>> mSubs; // This is a list of the subscribers to update the widget
  vtkSmartPointer<vtkMRMLROS2NODENode> mROS2Node; // proper MRML node // Moved to public which might be wrong!!
  void AddTransformForMatrix(vtkSmartPointer<vtkMatrix4x4> mat, std::string name);
  void updateMRMLSceneFromSubs(void);

  // bool testSubNode( vtkMRMLNode* node );
  // vtkMRMLROS2SubscriberNode* CreateSubscriberNode();

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

  KDL::ChainFkSolverPos_recursive * mKDLSolver = 0;
  size_t mKDLChainSize = 0;
  std::vector<vtkSmartPointer<vtkMRMLTransformNode> > mChainNodeTransforms;

  void ModelParameterCallback(std::shared_future<std::vector<rclcpp::Parameter>> future);
  std::shared_ptr<rclcpp::Node> mNodePointer;
  std::shared_ptr<rclcpp::AsyncParametersClient> mParameterClient;
  // vtkSmartPointer<vtkMRMLROS2NODENode> mROS2Node; // proper MRML node
  bool parameterNodeCallbackFlag = false;
  std::vector<std::string> link_names_vector;
  std::vector<std::string> link_parent_names_vector;

  // state
  struct {
    bool IsUsingTopic = false; // if not topic, using tf
    bool sendingTf = false;
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

  vtkSmartPointer<vtkMRMLNode> mTestSubscriber;



  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> mJointStateSubscription;
  void JointStateCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);

  std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> mTfListener;
  void queryTfNode(void);
  void updateTransformFromTf(geometry_msgs::msg::TransformStamped transformStamped, int transformCount);

  // Set up the broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;

  void initializeFkSolver(void);
};

#endif
