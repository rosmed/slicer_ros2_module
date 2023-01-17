#include <vtkMRMLROS2Tf2BufferNode.h>
#include <vtkMRMLROS2Tf2BufferInternals.h>
#include <vtkMRMLROS2Tf2LookupNode.h>
#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2NodeInternals.h>
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

  mInternals->mNodePointer = rosNodePtr->mInternals->mNodePointer;
  vtkMRMLROS2Tf2BufferNode * buffer = rosNodePtr->GetBufferNodeByID(this->GetID()); // check if ptr is set ( don't need to check ID ) - should also have a GetBuffer method
  if (buffer != nullptr){
    errorMessage = "This buffer has already been added to the ROS2 node.";
    return false;
  }

  mInternals->mTfBuffer = std::make_unique<tf2_ros::Buffer>(mInternals->mNodePointer->get_clock());
  mInternals->mTfListener = std::make_shared<tf2_ros::TransformListener>(*mInternals->mTfBuffer);
  rosNodePtr->SetNthNodeReferenceID("Tf2buffers",
				      rosNodePtr->GetNumberOfNodeReferences("Tf2buffers"),
				      this->GetID());
  this->SetNodeReferenceID("node", nodeId);
  rosNodePtr->mBuffer = this;
  return true;
}

bool vtkMRMLROS2Tf2BufferNode::AddLookupNode(vtkMRMLROS2Tf2LookupNode * lookupNode){
  
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 buffer MRML node needs to be added to the scene first");
    return false;
  }
  if(lookupNode->isParentAndChildSet()){
    mLookupNodes.push_back(lookupNode);
    this->SetNodeReferenceID("Lookups", lookupNode->GetID());
    return true;
  }
  else{
    vtkErrorMacro(<< "Lookup child and parent not set");
    return false;
  }
}

vtkMRMLROS2Tf2LookupNode * vtkMRMLROS2Tf2BufferNode::CreateAndAddLookupNode(const std::string & parent_id, const std::string & child_id){
  
  vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookupNode = vtkMRMLROS2Tf2LookupNode::New();
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 buffer MRML node needs to be added to the scene first");
    return nullptr;
  }
  this->GetScene()->AddNode(lookupNode);
  lookupNode->SetParentID(parent_id);
  lookupNode->SetChildID(child_id);
  if(lookupNode->isParentAndChildSet()){
    mLookupNodes.push_back(lookupNode);
    this->SetNodeReferenceID("Lookups", lookupNode->GetID());
    return lookupNode;
  }
  else{
    vtkErrorMacro(<< "Parent or child id is an empty string");
    return nullptr;
  }
  return lookupNode;
}

bool vtkMRMLROS2Tf2BufferNode::Spin(){ 
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 buffer MRML node needs to be added to the scene first");
    return false;
  }
  for (size_t index = 0; index < mLookupNodes.size(); ++index) {
      vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookupNode = mLookupNodes[index];
      if(lookupNode->isParentAndChildSet()){
        std::string errorMessage;
        vtkMRMLTransformNode *transform = vtkMRMLTransformNode::SafeDownCast(scene->GetNodeByID(lookupNode->GetID()));
        if (!transform){
          vtkErrorMacro(<< "Transform does not exist for provided ID.");
          return false;
        } 
        if (!mInternals->lookupTryCatch(lookupNode->GetParentID(), lookupNode->GetChildID(), errorMessage, lookupNode, transform)) {
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

void vtkMRMLROS2Tf2BufferNode::WriteXML( ostream& of, int nIndent ) // double check save and restore - lookups should have a reference role to the buffer - try save and restore - open module should trigger spin
// will need to define the roles 
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
