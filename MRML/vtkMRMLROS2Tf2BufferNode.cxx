// Slicer includes 
#include <vtkObject.h>
#include <vtkMRMLTransformNode.h>
#include <vtkEventBroker.h>
#include <vtkMRMLScene.h>

// SlicerROS2 includes
#include <vtkMRMLROS2Tf2BufferNode.h>
#include <vtkMRMLROS2Tf2BufferInternals.h>
#include <vtkMRMLROS2Tf2LookupNode.h>
#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2NodeInternals.h>
#include <vtkROS2ToSlicer.h>


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

  // Check if the node is in the scene
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 buffer MRML node needs to be added to the scene first");
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

  // Add the buffer to the ros2 node
  mInternals->mNodePointer = rosNodePtr->mInternals->mNodePointer;
  vtkMRMLROS2Tf2BufferNode * buffer = rosNodePtr->GetBuffer();
  if (buffer != nullptr){
    vtkErrorMacro(<< "This buffer has already been added to the ROS2 node.");
    return false;
  }
  mInternals->mTfBuffer = std::make_unique<tf2_ros::Buffer>(mInternals->mNodePointer->get_clock());
  mInternals->mTfListener = std::make_shared<tf2_ros::TransformListener>(*mInternals->mTfBuffer);
  rosNodePtr->SetNthNodeReferenceID("buffer",
				      rosNodePtr->GetNumberOfNodeReferences("buffer"),
				      this->GetID());
  this->SetNodeReferenceID("node", nodeId);
  rosNodePtr->mBuffer = this;
  return true;
}

bool vtkMRMLROS2Tf2BufferNode::AddLookupNode(vtkMRMLROS2Tf2LookupNode * lookupNode){
  // Check that the buffer is in the scene
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 buffer MRML node needs to be added to the scene first");
    return false;
  }
  // Check if the lookup node is already in the list of lookups
  if (GetLookupNodeByID(lookupNode->GetID()) != nullptr){
    vtkErrorMacro(<< "This lookup node has been added to the buffer already.");
    return false;
  }
  // Check that the parent and child of the lookup are set and then add to the buffer node and assign references
  if(lookupNode->isParentAndChildSet()){
    mLookupNodes.push_back(lookupNode); // Add the lookup node to the list of refences
    this->SetNodeReferenceID("lookups", lookupNode->GetID()); // Add both references
    lookupNode->SetNodeReferenceID("buffer", this->GetID()); 
    return true;
  }
  else{
    vtkErrorMacro(<< "Lookup child and parent are not set yet.");
    return false;
  }
}

vtkMRMLROS2Tf2LookupNode * vtkMRMLROS2Tf2BufferNode::CreateAndAddLookupNode(const std::string & parent_id, const std::string & child_id){
  
  // Check the buffer node is in the scene
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 buffer MRML node needs to be added to the scene first");
    return nullptr;
  }

  // Create the lookup node
  vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookupNode = vtkMRMLROS2Tf2LookupNode::New();
  // Add the node to the scene
  this->GetScene()->AddNode(lookupNode);

  // Set the parent and child id
  if (!lookupNode->SetParentID(parent_id)){
    vtkErrorMacro(<< "Parent ID is an empty string.");
    return nullptr;
  }
  if (!lookupNode->SetChildID(child_id)){
    vtkErrorMacro(<< "Child ID is an empty string.");
    return nullptr;
  }

  // Add to list of lookup nodes and assigned references
  mLookupNodes.push_back(lookupNode);
  // this->SetNodeReferenceID("lookups", lookupNode->GetID());
  lookupNode->SetNodeReferenceID("buffer", this->GetID());

  this->SetNthNodeReferenceID("lookups",
				    this->GetNumberOfNodeReferences("lookups"),
				    lookupNode->GetID());
  
  return lookupNode;
}

vtkMRMLROS2Tf2LookupNode * vtkMRMLROS2Tf2BufferNode::GetLookupNodeByID(const std::string & nodeID){

  // Go through list of lookup nodes and check for duplicate IDs
  for (auto lookupNode: mLookupNodes){
    const std::string lookupID = lookupNode->GetID();
    if (lookupID == nodeID){
      return lookupNode;
    }
  }
  return nullptr;
}

bool vtkMRMLROS2Tf2BufferNode::Spin(){ 
  
  // Make sure buffer node is still in the scene
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 buffer MRML node needs to be added to the scene first");
    return false;
  }
  // iterate through lookup nodes - make sure they have parent and children set and call lookup try catch
  for (auto lookupNode: mLookupNodes){
      if(lookupNode->isParentAndChildSet()){
        if (!LookupTryCatch(lookupNode->GetParentID(), lookupNode->GetChildID(), lookupNode)) {
          vtkErrorMacro(<< "Transform exception"); 
        }
      }
  }
  return true;
}

bool vtkMRMLROS2Tf2BufferNode::LookupTryCatch(const std::string & parent_id, const std::string & child_id, vtkMRMLROS2Tf2LookupNode * lookupNode)
  { 
    try {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped = mInternals->mTfBuffer->lookupTransform(parent_id, child_id, tf2::TimePointZero);  // check how old we want the data to be (right now it's doing it no matter how old) - for now we don't care
      vtkNew<vtkMatrix4x4> matrix;
      vtkROS2ToSlicer(transformStamped, matrix);
      lookupNode->SetMatrixTransformToParent(matrix);
      lookupNode->Modified();
      return true;
    } 
    catch (tf2::TransformException & ex) {
      vtkErrorMacro(<< "Could not find the transform between " + parent_id + " and " + child_id); 
      return false;
    }
    catch(...){
      vtkErrorMacro(<< "Got an undefined exception.");
      return false;
    }
    return false;
  }

void vtkMRMLROS2Tf2BufferNode::WriteXML( ostream& of, int nIndent ) 
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(BufferNodeName, BufferNodeName);
  vtkMRMLWriteXMLEndMacro();
}

void vtkMRMLROS2Tf2BufferNode::ReadXMLAttributes( const char** atts )
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(BufferNodeName, BufferNodeName);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}
