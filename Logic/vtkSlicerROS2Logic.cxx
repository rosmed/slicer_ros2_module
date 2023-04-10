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
  // prevent saving the default node in the scene as it is created automatically on startup
  mDefaultROS2Node->SaveWithSceneOff();
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
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2SubscriberIntNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2SubscriberFloatNode>::New());
  
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2SubscriberIntArrayNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2SubscriberFloatArrayNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2SubscriberIntNArrayNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2SubscriberPoseStampedNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2SubscriberJoyNode>::New());
  // Publishers
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherStringNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherBoolNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherIntNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherFloatNode>::New());
  
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherIntArrayNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherFloatArrayNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherIntNArrayNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherPoseStampedNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherWrenchStampedNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherPoseArrayNode>::New());
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2PublisherCartesianImpedanceGainsNode>::New());
  // Parameters
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2ParameterNode>::New());
  // Tf2
  this->GetMRMLScene()->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2Tf2BroadcasterNode>::New());
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


vtkMRMLROS2NodeNode * vtkSlicerROS2Logic::GetDefaultROS2Node(void) const
{
  return mDefaultROS2Node;
}


void vtkSlicerROS2Logic::AddRobot(const std::string & parameterNodeName, const std::string & parameterName, const std::string & robotName)
{
  vtkSmartPointer<vtkMRMLROS2RobotNode> robot = vtkMRMLROS2RobotNode::New();
  this->GetMRMLScene()->AddNode(robot);
  robot->AddToROS2Node(mDefaultROS2Node->GetID(), parameterNodeName, parameterName);
  robot->SetRobotName(robotName);
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
