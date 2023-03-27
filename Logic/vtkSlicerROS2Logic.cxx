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


// SlicerROS2 Logic includes
#include <vtkSlicerROS2Logic.h>
#include <qSlicerCoreApplication.h>

// VTK includes
#include <vtkTimerLog.h>

// MRML includes
#include <vtkMRMLScene.h>
#include <vtkMRMLModelNode.h>
#include <vtkMRMLTransformStorageNode.h>
#include <vtkMRMLTransformNode.h>
#include <vtkMRMLLinearTransformNode.h>
#include <vtkMRMLModelStorageNode.h>
#include <vtkMRMLDisplayNode.h>
#include <vtkMRMLModelDisplayNode.h>

// MRMLROS2
#include <vtkMRMLROS2Utils.h>
#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2SubscriberDefaultNodes.h>
#include <vtkMRMLROS2PublisherDefaultNodes.h>
#include <vtkMRMLROS2ParameterNode.h>
#include <vtkMRMLROS2Tf2BroadcasterNode.h>
#include <vtkMRMLROS2Tf2BufferNode.h>
#include <vtkMRMLROS2Tf2LookupNode.h>
#include <vtkMRMLROS2RobotNode.h>


//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlicerROS2Logic);


//----------------------------------------------------------------------------
vtkSlicerROS2Logic::vtkSlicerROS2Logic()
{
  mTimerLog = vtkSmartPointer<vtkTimerLog>::New();
  vtkMRMLROS2::ROSInit();
}


//----------------------------------------------------------------------------
vtkSlicerROS2Logic::~vtkSlicerROS2Logic()
{
  vtkMRMLROS2::ROSShutdown();
}


//----------------------------------------------------------------------------
void vtkSlicerROS2Logic::PrintSelf(std::ostream & os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}


//---------------------------------------------------------------------------
void vtkSlicerROS2Logic::SetMRMLSceneInternal(vtkMRMLScene * newScene)
{
  vtkNew<vtkIntArray> events;
  events->InsertNextValue(vtkMRMLScene::NodeAddedEvent);
  events->InsertNextValue(vtkMRMLScene::NodeRemovedEvent);
  events->InsertNextValue(vtkMRMLScene::EndBatchProcessEvent);
  this->SetAndObserveMRMLSceneEventsInternal(newScene, events.GetPointer());

  mDefaultROS2Node = vtkMRMLROS2NodeNode::New();
  this->GetMRMLScene()->AddNode(mDefaultROS2Node);
  mDefaultROS2Node->Create("slicer");
  mROS2Nodes.push_back(mDefaultROS2Node);
}

//-----------------------------------------------------------------------------
void vtkSlicerROS2Logic::RegisterNodes(void)
{
  assert(this->GetMRMLScene() != 0);
  // ROS2 node
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2NodeNode>::New());
  // Subscribers
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2SubscriberStringNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2SubscriberBoolNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2SubscriberPoseStampedNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2SubscriberJoyNode>::New());
  // Publishers
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherStringNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherPoseStampedNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherWrenchStampedNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherPoseArrayNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherCartesianImpedanceGainsNode>::New());
  // Parameters
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2ParameterNode>::New());
  // Tf2
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2Tf2BroadcasterNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2Tf2BufferNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2Tf2LookupNode>::New());
  // Robot
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2RobotNode>::New());
}


//---------------------------------------------------------------------------
void vtkSlicerROS2Logic::UpdateFromMRMLScene(void)
{
}


//---------------------------------------------------------------------------
void vtkSlicerROS2Logic::OnMRMLSceneNodeAdded(vtkMRMLNode * node)
{
  vtkMRMLROS2NodeNode * rosNode = dynamic_cast<vtkMRMLROS2NodeNode *>(node);
  if (rosNode != nullptr) {
    if (std::find(mROS2Nodes.begin(), mROS2Nodes.end(), node) == mROS2Nodes.end()) {
      mROS2Nodes.push_back(rosNode);
    }
  }
}


//---------------------------------------------------------------------------
void vtkSlicerROS2Logic::OnMRMLSceneNodeRemoved(vtkMRMLNode* node)
{
  vtkMRMLROS2NodeNode * rosNode = dynamic_cast<vtkMRMLROS2NodeNode *>(node);
  if (rosNode != nullptr) {
    auto it = std::find(mROS2Nodes.begin(), mROS2Nodes.end(), node);
    if (it != mROS2Nodes.end()) {
      mROS2Nodes.erase(it);
    }
  }
}


void vtkSlicerROS2Logic::Spin(void)
{
  mTimerLog->StartTimer();
  SlicerRenderBlocker renderBlocker;
  for (auto & n : mROS2Nodes) {
    // check if output of GetROS2NodeName() is equal to "undefined" -> This implies that Create() has not been called yet
    if (n->GetROS2NodeName() == "undefined") continue; // FIXME: This is a hack to prevent a crash when the ROS2 node is not created yet
    n->Spin();
  }
  mTimerLog->StopTimer();
  // std::cout << mTimerLog->GetElapsedTime() * 1000.0 << "ms" << std::endl; - commented out for development
}


vtkSmartPointer<vtkMRMLROS2NodeNode> vtkSlicerROS2Logic::GetDefaultROS2Node(void) const
{
  return mDefaultROS2Node;
}


void vtkSlicerROS2Logic::AddSomePublishers(void)
{
  // the long way
  vtkSmartPointer<vtkMRMLROS2PublisherStringNode> stringPub = vtkMRMLROS2PublisherStringNode::New();
  this->GetMRMLScene()->AddNode(stringPub);
  stringPub->AddToROS2Node(mDefaultROS2Node->GetID(), "/string_pub");
  // the fast way
  mDefaultROS2Node->CreateAndAddPublisher("vtkMRMLROS2PublisherStringNode", "/string_pub_2");
}


void vtkSlicerROS2Logic::AddSomeSubscribers(void)
{
  // the long way
  vtkSmartPointer<vtkMRMLROS2SubscriberStringNode> subString = vtkMRMLROS2SubscriberStringNode::New();
  this->GetMRMLScene()->AddNode(subString);
  subString->AddToROS2Node(mDefaultROS2Node->GetID(), "/string_sub");
  // with VTK type
  vtkSmartPointer<vtkMRMLROS2SubscriberPoseStampedNode> subPose = vtkMRMLROS2SubscriberPoseStampedNode::New();
  this->GetMRMLScene()->AddNode(subPose);
  subPose->AddToROS2Node(mDefaultROS2Node->GetID(), "/pose_sub");
  // the fast way
  mDefaultROS2Node->CreateAndAddSubscriber("vtkMRMLROS2SubscriberStringNode", "/string_sub_2");
}


void vtkSlicerROS2Logic::AddSomeParameters(void)
{
  // the long way
  vtkSmartPointer<vtkMRMLROS2ParameterNode> param = vtkMRMLROS2ParameterNode::New();
  this->GetMRMLScene()->AddNode(param);
  param->AddToROS2Node(mDefaultROS2Node->GetID(), "/dummy_node_name");
}


void vtkSlicerROS2Logic::AddSomeTf2Nodes(void)
{
  vtkSmartPointer<vtkMRMLROS2Tf2BroadcasterNode> tfBroadcaster = vtkMRMLROS2Tf2BroadcasterNode::New();
  this->GetMRMLScene()->AddNode(tfBroadcaster);
  tfBroadcaster->AddToROS2Node(mDefaultROS2Node->GetID());
  // Add Tf2 Buffer
  // keep this as the long way
  vtkSmartPointer<vtkMRMLROS2Tf2BufferNode> tfBuffer = vtkMRMLROS2Tf2BufferNode::New();
  this->GetMRMLScene()->AddNode(tfBuffer);
  tfBuffer->AddToROS2Node(mDefaultROS2Node->GetID());
  // vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> tfLookup = vtkMRMLROS2Tf2LookupNode::New();
  // this->GetMRMLScene()->AddNode(tfLookup);
  // tfLookup->SetParentID("world");
  // tfLookup->SetChildID("turtle2");
  // tfBuffer->AddLookupNode(tfLookup);

  // add the short way
  // we're enforcing a single buffer
}


void vtkSlicerROS2Logic::AddRobot(const std::string & parameterNodeName, const std::string & parameterName, const std::string & robotName)
{
  // Sensable phantom, requires
  // ros2 launch sensable_omni_model omni.launch.py  -- to get the model
  // ros2 run sensable_omni_model pretend_omni_joint_state_publisher  -- wave the arm around
  vtkSmartPointer<vtkMRMLROS2RobotNode> robot = vtkMRMLROS2RobotNode::New();
  this->GetMRMLScene()->AddNode(robot);
  robot->AddToROS2Node(mDefaultROS2Node->GetID(), parameterNodeName, parameterName);
  robot->SetRobotName(robotName);


  // dVRK, requires
  // ros2 run dvrk_robot dvrk_console_json -j ~/ros2_ws/src/cisst-saw/sawIntuitiveResearchKit/share/console/console-PSM1_KIN_SIMULATED.json  -- to run fake PSM1
  // ros2 launch dvrk_model dvrk_state_publisher.launch.py arm:=PSM1  -- to get the model
  // ros2 run dvrk_python dvrk_arm_test.py -a PSM1  -- to make the PSM1 move around
  // vtkSmartPointer<vtkMRMLROS2RobotNode> robot2 = vtkMRMLROS2RobotNode::New();
  // this->GetMRMLScene()->AddNode(robot2);
  // robot2->AddToROS2Node(mDefaultROS2Node->GetID(), "/PSM1/robot_state_publisher", "robot_description");
}

void vtkSlicerROS2Logic::RemoveRobot(const std::string & robotName)
{
  // Get the robot from the node (first make sure it's the right one)
  int numRobots = mDefaultROS2Node->GetNumberOfNodeReferences("robot");
  for (int i = 0; i < numRobots; i++){
    vtkMRMLROS2RobotNode * robotNode = vtkMRMLROS2RobotNode::SafeDownCast(mDefaultROS2Node->GetNthNodeReference("robot", i));
    std::string currentRobot = robotNode->GetRobotName();
    if (currentRobot == robotName){
      // Remove the lookups on that robot
      int numLookups = robotNode->GetNumberOfNodeReferences("lookup");
      for (int i = 0; i < numLookups; i++) {
        vtkMRMLROS2Tf2LookupNode * lookupNode = vtkMRMLROS2Tf2LookupNode::SafeDownCast(robotNode->GetNthNodeReference("lookup", 0)); // always grab the first one because the ref id changes
        vtkMRMLModelNode * modelNode = vtkMRMLModelNode::SafeDownCast(robotNode->GetNthNodeReference("model", 0)); // always grab the first one because the ref id changes
        vtkMRMLROS2ParameterNode * parameterNode = vtkMRMLROS2ParameterNode::SafeDownCast(robotNode->GetNthNodeReference("ObservedParameter", 0)); // always grab the first one because the ref id changes
        this->GetMRMLScene()->RemoveNode(lookupNode);
        this->GetMRMLScene()->RemoveNode(modelNode);
        this->GetMRMLScene()->RemoveNode(parameterNode);
      }

      // Remove the robot itself
      this->GetMRMLScene()->RemoveNode(robotNode);
      return;
    }
  }
}
