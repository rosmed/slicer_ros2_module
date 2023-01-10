#include <vtkMRMLROS2Tf2BufferNode.h>
#include <vtkMRMLROS2Tf2BufferInternals.h>
#include <vtkMRMLROS2Tf2LookupNode.h>
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
    vtkErrorMacro(<< "AddToROS2Node, tf2 buffer MRML node needs to be added to the scene first");
    return false;
  }
  std::string errorMessage;
  mBufferNode = this;
  if (!mInternals->AddToROS2Node(this, mBufferNode, scene, nodeId, errorMessage)) {
    vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
    return false;
  }
  return true;
}

bool vtkMRMLROS2Tf2BufferNode::AddLookupNode(vtkMRMLROS2Tf2LookupNode * lookupNode){
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 buffer MRML node needs to be added to the scene first");
    return false;
  }
  if(lookupNode->CheckIfParentAndChildSet()){
    mLookupNodes.push_back(lookupNode);
    this->SetNodeReferenceID("Lookups", lookupNode->GetID());
    return true;
  }
}

bool vtkMRMLROS2Tf2BufferNode::InitiateLookup(){
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 buffer MRML node needs to be added to the scene first");
    return false;
  }
  for (size_t index = 0; index < mLookupNodes.size(); ++index) {
      vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookupNode = mLookupNodes[index];
      if(lookupNode->CheckIfParentAndChildSet()){
        std::string errorMessage;
        if (!mInternals->InstantiateLookups(scene, lookupNode->GetParentID(), lookupNode->GetChildID(), errorMessage, lookupNode)) {
          vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
          return false;
        }
        else{
          mLookupNodes.push_back(lookupNode);
          this->SetNodeReferenceID("Lookups", lookupNode->GetID());
          return true;
        }
      }
  }
}

void vtkMRMLROS2Tf2BufferNode::WriteXML( ostream& of, int nIndent )
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2Tf2BufferNode::ReadXMLAttributes( const char** atts )
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}
