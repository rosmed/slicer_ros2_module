#include <vtkMRMLScene.h>
#include <vtkMRMLROS2RobotNode.h>

vtkStandardNewMacro(vtkMRMLROS2RobotNode);

vtkMRMLNode * vtkMRMLROS2RobotNode::CreateNodeInstance(void)
{
  return SelfType::New();

}

const char * vtkMRMLROS2RobotNode::GetNodeTagName(void)
{
  return "ROS2Node";
}

vtkMRMLROS2RobotNode::vtkMRMLROS2RobotNode()
{
//   mInternals = std::make_unique<vtkMRMLROS2NodeInternals>();
}

vtkMRMLROS2RobotNode::~vtkMRMLROS2RobotNode()
{
}

// vtkMRMLROS2RobotNode::InitializeRobotDescription()
// {
// }

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
//   vtkMRMLWriteXMLStdStringMacro(ROS2NodeName, ROS2NodeName);
  vtkMRMLWriteXMLEndMacro();
}

void vtkMRMLROS2RobotNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
//   vtkMRMLReadXMLStdStringMacro(ROS2NodeName, ROS2NodeName);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}
