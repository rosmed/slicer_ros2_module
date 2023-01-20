#include <vtkMRMLScene.h>
#include <vtkMRMLROS2RobotNode.h>
#include <vtkMRMLROS2ParameterNode.h>
#include <vtkMRMLScene.h>
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

bool vtkMRMLROS2RobotNode::InitializeRobotDescription()
{
  // Check if the node is in the scene
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 buffer MRML node needs to be added to the scene first");
    return false;
  }
  

  vtkSmartPointer<vtkMRMLROS2ParameterNode> param = vtkMRMLROS2ParameterNode::New();
  this->GetScene()->AddNode(param);
  param->AddToROS2Node(mROS2Node->GetID(), "/robot_state_publisher");
  param->AddParameter("robot_description");
  param->PrintParameterValue("robot_description");
  mRobotDescriptionParameterNode = param;
  return true;
}

void vtkMRMLROS2RobotNode::PrintRobotDescription()
{
  std::string paramValue;
  mRobotDescriptionParameterNode->PrintParameterValue("robot_description", paramValue);
  std::cerr << "Parameter value:" << paramValue << std::endl;
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
