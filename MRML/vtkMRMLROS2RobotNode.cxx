#include <vtkMRMLScene.h>
#include <vtkMRMLROS2RobotNode.h>
#include <vtkMRMLROS2ParameterNode.h>
#include <vtkMRMLScene.h>
#include <vtkEventBroker.h>
#include <vtkMRMLROS2NODENode.h>

vtkStandardNewMacro(vtkMRMLROS2RobotNode);

vtkMRMLNode * vtkMRMLROS2RobotNode::CreateNodeInstance(void)
{
  return SelfType::New();

}

const char * vtkMRMLROS2RobotNode::GetNodeTagName(void)
{
  return "ROS2RobotNode";
}

vtkMRMLROS2RobotNode::vtkMRMLROS2RobotNode()
{
}

vtkMRMLROS2RobotNode::~vtkMRMLROS2RobotNode()
{
}

bool vtkMRMLROS2RobotNode::AddToROS2Node(const char * nodeId)
{
  this->SetName(mMRMLNodeName.c_str());

  // Check if the node is in the scene
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, robot MRML node needs to be added to the scene first");
    return false;
  }
  
  // Check that the ROS2 node node is in the scene and of the correct type
  vtkMRMLNode * rosNodeBasePtr = scene->GetNodeByID(nodeId);
  if (!rosNodeBasePtr) {
    vtkErrorMacro(<< "Unable to locate ros2 node in the scene");
    return false;
  }
  vtkMRMLROS2NODENode * rosNodePtr = dynamic_cast<vtkMRMLROS2NODENode *>(rosNodeBasePtr);
  if (!rosNodePtr) {
    vtkErrorMacro(<< std::string(rosNodeBasePtr->GetName()) + " doesn't seem to be a vtkMRMLROS2NODENode");
    return false;
  }

  // Add the robot to the ros2 node
  rosNodePtr->SetNthNodeReferenceID("robot",
				      rosNodePtr->GetNumberOfNodeReferences("robot"),
				      this->GetID());
  this->SetNodeReferenceID("node", nodeId);
  mROS2Node = rosNodePtr;
  return true;
}

bool vtkMRMLROS2RobotNode::SetRobotDescriptionParameterNode(vtkMRMLROS2ParameterNode * param)
{
  // Check if the node is in the scene
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 buffer MRML node needs to be added to the scene first");
    return false;
  }
  mRobotDescriptionParameterNode = param;
  ObserveParameterNode(param);
  return true;
}

// robot node - create paramater
// add observer to parameter node and check when it's ready -> Call IsParameterSet() - paramater node has multiple params that it tracks
// has a callback that uses paramater (robot description)  to parse urdf
// should set t

void vtkMRMLROS2RobotNode::PrintRobotDescription()
{
  std::string paramValue;
  mRobotDescriptionParameterNode->PrintParameterValue("robot_description", paramValue);
  std::cerr << "Parameter value:" << paramValue << std::endl;
}

void vtkMRMLROS2RobotNode::ObserveParameterNode(vtkMRMLROS2ParameterNode * node )
{
  if (!this->GetScene()->GetNodeByID(node->GetID())){
    vtkErrorMacro(<< "Transform is not in the scene.");
    return;
  }
  node->AddObserver(vtkMRMLROS2ParameterNode::ParameterModifiedEvent, this, &vtkMRMLROS2RobotNode::ObserveParameterNodeCallback);
  this->SetAndObserveNodeReferenceID("ObservedParameter", node->GetID());
}

void vtkMRMLROS2RobotNode::ObserveParameterNodeCallback( vtkObject* caller, unsigned long, void* vtkNotUsed(callData))
{
  vtkMRMLROS2ParameterNode* parameterNode = vtkMRMLROS2ParameterNode::SafeDownCast(caller);
  if (!parameterNode)
  {
    return;
  }
  else
  {
    std::cerr << "Parameter node is modified." << std::endl; // for debugging
    // ParseRobotDescription(parameterNode);
  }
}

void vtkMRMLROS2RobotNode::SetRobotName(const std::string & robotName)
{
  mROS2RobotName = robotName;
}

void vtkMRMLROS2RobotNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
}

void vtkMRMLROS2RobotNode::WriteXML( ostream& of, int nIndent )
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(ROS2RobotName, ROS2RobotName);
  vtkMRMLWriteXMLEndMacro();
}

void vtkMRMLROS2RobotNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(ROS2RobotName, ROS2RobotName);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}
