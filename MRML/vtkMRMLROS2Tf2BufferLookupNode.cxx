#include <vtkMRMLROS2Tf2BufferLookupNode.h>
#include <vtkMRMLROS2Tf2BufferLookupInternals.h>
#include <vtkSlicerToROS2.h>
#include <vtkObject.h>
#include <vtkMRMLTransformNode.h>
#include <vtkEventBroker.h>

vtkStandardNewMacro(vtkMRMLROS2Tf2BufferLookupNode);

vtkMRMLROS2Tf2BufferLookupNode::vtkMRMLROS2Tf2BufferLookupNode()
{
  mInternals = std::make_unique<vtkMRMLROS2Tf2BufferLookupInternals>();
}

vtkMRMLROS2Tf2BufferLookupNode::~vtkMRMLROS2Tf2BufferLookupNode()
{
}

vtkMRMLNode * vtkMRMLROS2Tf2BufferLookupNode::CreateNodeInstance(void)
{
  return SelfType::New();

}

const char * vtkMRMLROS2Tf2BufferLookupNode::GetNodeTagName(void)
{
  return "ROS2Tf2Buffer";
}

void vtkMRMLROS2Tf2BufferLookupNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
}

bool vtkMRMLROS2Tf2BufferLookupNode::AddToROS2Node(const char * nodeId)
{
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 broadcaster MRML node needs to be added to the scene first");
    return false;
  }
  std::string errorMessage;
  if (!mInternals->AddToROS2Node(this, scene, nodeId, errorMessage)) {
    vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
    return false;
  }
  return true;
}

void vtkMRMLROS2Tf2BufferLookupNode::SetParentID(const std::string & parent_id)
{
  mParentID = parent_id;
  UpdateMRMLNodeName();
}

std::string vtkMRMLROS2Tf2BufferLookupNode::GetParentID()
{
  return mParentID;
}

void vtkMRMLROS2Tf2BufferLookupNode::SetChildID(const std::string & child_id)
{
  mChildID = child_id;
  UpdateMRMLNodeName();
}

std::string vtkMRMLROS2Tf2BufferLookupNode::GetChildID()
{
  return mChildID;
}

bool vtkMRMLROS2Tf2BufferLookupNode::CheckIfParentAndChildSet()
{
  if (mParentID.empty() || mChildID.empty()){
    return false;
  }
  else{
    return true; 
  }
}

void vtkMRMLROS2Tf2BufferLookupNode::UpdateMRMLNodeName()
{
  std::string mMRMLNodeName = "ros2:tf2bufferlookup:" + mParentID + "To" + mChildID;
  if (mParentID.empty() || mChildID.empty()){
    std::string emptyName = "ros2:tf2bufferlookup:empty";
    this->SetName(emptyName.c_str());
    return;
  }
  this->SetName(mMRMLNodeName.c_str());
}

void vtkMRMLROS2Tf2BufferLookupNode::WriteXML( ostream& of, int nIndent )
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(mChildID, ChildID);
  vtkMRMLWriteXMLStdStringMacro(mParentID, ParentID);
  vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2Tf2BufferLookupNode::ReadXMLAttributes( const char** atts )
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(mChildID, ChildID);
  vtkMRMLReadXMLStdStringMacro(mParentID, ParentID);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}
