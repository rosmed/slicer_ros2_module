
#include <vtkMRMLROS2Tf2LookupNode.h>

#include <vtkObject.h>
#include <vtkEventBroker.h>

#include <vtkMRMLScene.h>
#include <vtkMRMLTransformNode.h>

#include <vtkMRMLROS2Utils.h>
#include <vtkMRMLROS2NodeNode.h>

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


void vtkMRMLROS2Tf2LookupNode::PrintSelf(ostream & os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
}


bool vtkMRMLROS2Tf2LookupNode::SetParentID(const std::string & parent_id)
{
  if (parent_id.empty()) {
    vtkErrorMacro(<< "SetParentID: parent ID cannot be empty string");
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
    vtkErrorMacro(<< "SetChildID: child ID cannot be empty string");
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
  if (mParentID.empty() || mChildID.empty()) {
    return false;
  }
  else {
    return true;
  }
}


bool vtkMRMLROS2Tf2LookupNode::AddToROS2Node(const char * nodeId)
{
  // Check that the lookup is in the scene
  this->SetName(mMRMLNodeName.c_str());
  std::string errorMessage;
  vtkMRMLROS2NodeNode * mrmlROSNodePtr = vtkMRMLROS2::CheckROS2NodeExists(this, nodeId, errorMessage);
  if (!mrmlROSNodePtr) {
      vtkErrorMacro(<< "AddToROS2Node: " << errorMessage);
      return false;
  }

  // Check that the buffer hasn't already been added to the node
  vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookup = mrmlROSNodePtr->GetTf2LookupNodeByID(this->GetID());
  if ((lookup != nullptr) && lookup->IsAddedToROS2Node()) {
    vtkErrorMacro(<< "AddToROS2Node: this lookup has already been added to the ROS2 node.");
    return false;
  }

  // Add the lookup to the node and set up references
  mrmlROSNodePtr->SetTf2Buffer();
  mAddedToROS2Node = true;
  mrmlROSNodePtr->SetNthNodeReferenceID("lookup",
                                        mrmlROSNodePtr->GetNumberOfNodeReferences("lookup"),
                                        this->GetID());
  this->SetNodeReferenceID("node", nodeId);
  mrmlROSNodePtr->WarnIfNotSpinning("adding tf2 lookup for \"" + mMRMLNodeName + "\"");
  return true;
}


bool vtkMRMLROS2Tf2LookupNode::IsAddedToROS2Node(void) const
{
  return mAddedToROS2Node;
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
  int nbNodeRefs = this->GetNumberOfNodeReferences("node");
  if (nbNodeRefs == 0) {
    // assigned to the default ROS node
    auto defaultNode = scene->GetFirstNodeByName("ros2:node:slicer");
    if(!defaultNode){
      vtkErrorMacro(<< "UpdateScene: default ros2 node unavailable. Unable to set reference for broadcaster \"" << GetName() << "\"");
      return;
    }
    this->AddToROS2Node(defaultNode->GetID());
  } else if (nbNodeRefs == 1) {
    this->AddToROS2Node(this->GetNthNodeReference("node", 0)->GetID());
  } else {
    vtkErrorMacro(<< "UpdateScene: more than one ROS2 node reference defined for broadcaster \"" << GetName() << "\"");
  }
}
