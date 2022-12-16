#include <vtkMRMLROS2Tf2BroadcasterNode.h>
#include <vtkMRMLROS2Tf2BroadcasterInternals.h>
#include "vtkCommand.h"
#include <vtkSlicerToROS2.h>

vtkStandardNewMacro(vtkMRMLROS2Tf2BroadcasterNode);

vtkMRMLNode * vtkMRMLROS2Tf2BroadcasterNode::CreateNodeInstance(void)
{
  return SelfType::New();

}


const char * vtkMRMLROS2Tf2BroadcasterNode::GetNodeTagName(void)
{
  return "ROS2Tf2Broadcaster";
}


vtkMRMLROS2Tf2BroadcasterNode::vtkMRMLROS2Tf2BroadcasterNode()
{
  mInternals = std::make_unique<vtkMRMLROS2Tf2BroadcasterInternals>();
}

vtkMRMLROS2Tf2BroadcasterNode::~vtkMRMLROS2Tf2BroadcasterNode()
{
}

void vtkMRMLROS2Tf2BroadcasterNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
}

bool vtkMRMLROS2Tf2BroadcasterNode::AddToROS2Node(const char * nodeId)
{
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 broadcaster MRML node needs to be added to the scene first");
    return false;
  }
  std::string errorMessage;
  if (!mInternals->AddToROS2Node(scene, nodeId, errorMessage)) {
    vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
    return false;
  }
  return true;
}

size_t vtkMRMLROS2Tf2BroadcasterNode::Broadcast(vtkMRMLTransformNode * message, const std::string & parent_id, const std::string & child_id)
{
  std::string errorMessage;
  if (!mInternals->Broadcast(message, parent_id, child_id)) {
    vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
    return false;
  }
  return true;
}


size_t vtkMRMLROS2Tf2BroadcasterNode::Broadcast(vtkMatrix4x4 * message, const std::string & parent_id, const std::string & child_id)
{
  std::string errorMessage;
  if (!mInternals->Broadcast(message, parent_id, child_id)) {
    vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
    return false;
  }
  return true;
}


void vtkMRMLROS2Tf2BroadcasterNode::WriteXML( ostream& of, int nIndent )
{
  // Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  // vtkIndent indent(nIndent);

  // vtkMRMLWriteXMLBeginMacro(of);
  // vtkMRMLWriteXMLStdStringMacro(ROS2NodeName, ROS2NodeName);
  // vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2Tf2BroadcasterNode::ReadXMLAttributes( const char** atts )
{
  // int wasModifying = this->StartModify();
  // Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  // vtkMRMLReadXMLBeginMacro(atts);
  // vtkMRMLReadXMLStdStringMacro(ROS2NodeName, ROS2NodeName);
  // vtkMRMLReadXMLEndMacro();
  // this->EndModify(wasModifying);

  // // This is created before UpdateScene() for all other nodes is called.
  // // It handles cases where Publishers and Subscribers are Read before the ROS2Node
  // this->Create(mROS2NodeName,false);
}
