#include <vtkMRMLROS2Tf2BufferNode.h>
#include <vtkMRMLROS2Tf2BufferInternals.h>
#include <vtkSlicerToROS2.h>
#include <vtkObject.h>
#include <vtkMRMLTransformNode.h>
#include <vtkEventBroker.h>

vtkStandardNewMacro(vtkMRMLROS2Tf2BufferNode);

vtkMRMLROS2Tf2BufferNode::vtkMRMLROS2Tf2BufferNode()
{
  mInternals = std::make_unique<vtkMRMLROS2Tf2BufferInternals>();
}

vtkMRMLROS2Tf2BufferNode::~vtkMRMLROS2Tf2BufferNode()
{
}

vtkMRMLNode * vtkMRMLROS2Tf2BufferNode::CreateNodeInstance(void)
{
  return SelfType::New();

}

const char * vtkMRMLROS2Tf2BufferNode::GetNodeTagName(void)
{
  return "ROS2Tf2Buffer";
}

void vtkMRMLROS2Tf2BufferNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
}

bool vtkMRMLROS2Tf2BufferNode::AddToROS2Node(const char * nodeId)
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

void vtkMRMLROS2Tf2BufferNode::SetParentID(const std::string & parent_id)
{
  mParentID = parent_id;
  UpdateMRMLNodeName();
}

std::string vtkMRMLROS2Tf2BufferNode::GetParentID()
{
  return mParentID;
}

void vtkMRMLROS2Tf2BufferNode::SetChildID(const std::string & child_id)
{
  mChildID = child_id;
  UpdateMRMLNodeName();
}

std::string vtkMRMLROS2Tf2BufferNode::GetChildID()
{
  return mChildID;
}

bool vtkMRMLROS2Tf2BufferNode::CheckIfParentAndChildSet()
{
  if (mParentID.empty() || mChildID.empty()){
    return false;
  }
  else{
    return true; 
  }
}

void vtkMRMLROS2Tf2BufferNode::UpdateMRMLNodeName()
{
  std::string mMRMLNodeName = "ros2:tf2buffer:" + mParentID + "To" + mChildID;
  if (mParentID.empty() || mChildID.empty()){
    std::string emptyName = "ros2:tf2buffer:empty";
    this->SetName(emptyName.c_str());
    return;
  }
  this->SetName(mMRMLNodeName.c_str());
}

bool vtkMRMLROS2Tf2BufferNode::AddLookupAndCreateNode()
{
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 broadcaster MRML node needs to be added to the scene first");
    return false;
  }
  if(CheckIfParentAndChildSet()){
    std::string errorMessage;
    if (!mInternals->AddLookupAndCreateNode(scene, mParentID, mChildID, errorMessage)) {
      vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
      return false;
    }
  }
}

bool vtkMRMLROS2Tf2BufferNode::AddLookupForExistingNode(const std::string transformID)
{
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 broadcaster MRML node needs to be added to the scene first");
    return false;
  }
  if(CheckIfParentAndChildSet()){
    std::string errorMessage;
    if (!mInternals->AddLookupForExistingNode(scene, mParentID, mChildID, transformID, errorMessage)) {
      vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
      return false;
    }
  }
}

void vtkMRMLROS2Tf2BufferNode::WriteXML( ostream& of, int nIndent )
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(mChildID, ChildID);
  vtkMRMLWriteXMLStdStringMacro(mParentID, ParentID);
  vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2Tf2BufferNode::ReadXMLAttributes( const char** atts )
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(mChildID, ChildID);
  vtkMRMLReadXMLStdStringMacro(mParentID, ParentID);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}
