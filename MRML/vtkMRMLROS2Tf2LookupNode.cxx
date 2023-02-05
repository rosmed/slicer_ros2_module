#include <vtkMRMLROS2Tf2LookupNode.h>
#include <vtkSlicerToROS2.h>
#include <vtkObject.h>
#include <vtkMRMLTransformNode.h>
#include <vtkEventBroker.h>
#include <vtkMRMLROS2Tf2BufferNode.h>
#include <vtkMRMLScene.h>

vtkStandardNewMacro(vtkMRMLROS2Tf2LookupNode);

vtkMRMLROS2Tf2LookupNode::vtkMRMLROS2Tf2LookupNode()
{
}


vtkMRMLROS2Tf2LookupNode::~vtkMRMLROS2Tf2LookupNode()
{
}


vtkMRMLNode * vtkMRMLROS2Tf2LookupNode::CreateNodeInstance(void)
{
  return SelfType::New();
}


const char * vtkMRMLROS2Tf2LookupNode::GetNodeTagName(void)
{
  return "ROS2Tf2Lookup";
}


void vtkMRMLROS2Tf2LookupNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
}


bool vtkMRMLROS2Tf2LookupNode::SetParentID(const std::string & parent_id)
{
  if (parent_id.empty()){
    vtkErrorMacro(<< "Parent ID cannot be empty string.");
    return false;
  }
  mParentID = parent_id;
  UpdateMRMLNodeName();
  return true;
}


const std::string& vtkMRMLROS2Tf2LookupNode::GetParentID(void) const
{
  return mParentID;
}


bool vtkMRMLROS2Tf2LookupNode::SetChildID(const std::string & child_id)
{
  if (child_id.empty()) {
    vtkErrorMacro(<< "Child ID cannot be empty string.");
    return false;
  }
  mChildID = child_id;
  UpdateMRMLNodeName();
  return true;
}


const std::string& vtkMRMLROS2Tf2LookupNode::GetChildID(void) const
{
  return mChildID;
}


bool vtkMRMLROS2Tf2LookupNode::IsParentAndChildSet(void) const
{
  if (mParentID.empty() || mChildID.empty()){
    return false;
  }
  else{
    return true;
  }
}


bool vtkMRMLROS2Tf2LookupNode::AddToBuffer(void)
{
  vtkSmartPointer<vtkMRMLROS2Tf2BufferNode> buffer = vtkMRMLROS2Tf2BufferNode::SafeDownCast(this->GetScene()->GetFirstNodeByClass("vtkMRMLROS2Tf2BufferNode"));
  if (buffer == nullptr) {
    vtkErrorMacro(<< "No buffer in the scene.");
    return false;
  }
  if (buffer->mLookupNodes.size() == 0) {
    buffer->AddLookupNode(this);
  }
  else {
    for (size_t j = 0; j < buffer->mLookupNodes.size(); j++) {
      auto lookupID = buffer->mLookupNodes[j]->GetID();
      if (lookupID == this->GetID()) {
        vtkErrorMacro(<< "Lookup node is already in the buffer list.");
        return false;
      }
      else {
        buffer->AddLookupNode(this);
        return true;
      }
    }
  }
  return false;
}


void vtkMRMLROS2Tf2LookupNode::SetModifiedOnLookup(const bool & set)
{
  mModifiedOnLookup = set;
}


bool vtkMRMLROS2Tf2LookupNode::GetModifiedOnLookup(void) const
{
  return mModifiedOnLookup;
}


void vtkMRMLROS2Tf2LookupNode::UpdateMRMLNodeName()
{
  std::string mMRMLNodeName = "ros2:tf2lookup:" + mParentID + "To" + mChildID;
  if (mParentID.empty() || mChildID.empty()) {
    std::string emptyName = "ros2:tf2lookup:empty";
    this->SetName(emptyName.c_str());
    return;
  }
  this->SetName(mMRMLNodeName.c_str());
}


void vtkMRMLROS2Tf2LookupNode::WriteXML(ostream& of, int nIndent)
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(mChildID, ChildID);
  vtkMRMLWriteXMLStdStringMacro(mParentID, ParentID);
  vtkMRMLWriteXMLBooleanMacro(mModifiedOnLookup, ModifiedOnLookup);
  vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2Tf2LookupNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(mChildID, ChildID);
  vtkMRMLReadXMLStdStringMacro(mParentID, ParentID);
  vtkMRMLReadXMLBooleanMacro(mModifiedOnLookup, ModifiedOnLookup);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}

void vtkMRMLROS2Tf2LookupNode::UpdateScene(vtkMRMLScene *scene)
{
  Superclass::UpdateScene(scene);
  this->AddToBuffer();
}
