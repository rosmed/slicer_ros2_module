#include <vtkMRMLROS2Tf2BroadcasterNode.h>
#include <vtkMRMLROS2Tf2BroadcasterInternals.h>
#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2NodeInternals.h>
#include <vtkSlicerToROS2.h>

// #include <vtkEventBroker.h>
// #include <vtkObject.h>

#include <vtkMRMLScene.h>
#include <vtkMRMLTransformNode.h>

vtkStandardNewMacro(vtkMRMLROS2Tf2BroadcasterNode);


vtkMRMLROS2Tf2BroadcasterNode::vtkMRMLROS2Tf2BroadcasterNode()
{
  mInternals = std::make_unique<vtkMRMLROS2Tf2BroadcasterInternals>();
}


vtkMRMLROS2Tf2BroadcasterNode::~vtkMRMLROS2Tf2BroadcasterNode()
{
}


vtkMRMLNode * vtkMRMLROS2Tf2BroadcasterNode::CreateNodeInstance(void)
{
  return SelfType::New();

}


const char * vtkMRMLROS2Tf2BroadcasterNode::GetNodeTagName(void)
{
  return "ROS2Tf2Broadcaster";
}


void vtkMRMLROS2Tf2BroadcasterNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os, indent);
}


bool vtkMRMLROS2Tf2BroadcasterNode::AddToROS2Node(const char * nodeId)
{
  // Check that the broadcaster is in the scene
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 broadcaster MRML node needs to be added to the scene first");
    return false;
  }

  // Check that the ROS2 node is in the scene
  vtkMRMLNode * rosNodeBasePtr = scene->GetNodeByID(nodeId);
  if (!rosNodeBasePtr) {
    vtkErrorMacro(<<"Unable to locate ros2 node in the scene");
    return false;
  }

  // Check that the ROS2 node is of the type that's expected
  vtkMRMLROS2NodeNode * rosNodePtr = dynamic_cast<vtkMRMLROS2NodeNode *>(rosNodeBasePtr);
  if (!rosNodePtr) {
    vtkErrorMacro(<< std::string(rosNodeBasePtr->GetName()) + " doesn't seem to be a vtkMRMLROS2NodeNode");
    return false;
  }

  // Check that the buffer hasn't already been added to the node
  vtkMRMLROS2Tf2BroadcasterNode * broadcaster = rosNodePtr->GetBroadcasterByID(this->GetID());
  if (broadcaster != nullptr) {
    vtkErrorMacro(<< "This buffer has already been added to the ROS2 node.");
    return false;
  }

  // Add the broadcaster to the node and set up references
  mInternals->mROSNode = rosNodePtr->mInternals->mNodePointer;
  mInternals->mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(mInternals->mROSNode);
  rosNodePtr->SetNthNodeReferenceID("broadcaster",
				    rosNodePtr->GetNumberOfNodeReferences("broadcaster"),
				    this->GetID());
  this->SetNodeReferenceID("node", nodeId);
  return true;
}


bool vtkMRMLROS2Tf2BroadcasterNode::SetParentID(const std::string & parent_id)
{
  if (parent_id.empty()) {
    vtkErrorMacro(<< "Parent ID cannot be empty string.");
    return false;
  }
  mParentID = parent_id;
  UpdateMRMLNodeName();
  return true;
}


const std::string & vtkMRMLROS2Tf2BroadcasterNode::GetParentID(void) const
{
  return mParentID;
}


bool vtkMRMLROS2Tf2BroadcasterNode::SetChildID(const std::string & child_id)
{
  if (child_id.empty()) {
    vtkErrorMacro(<< "Child ID cannot be empty string.");
    return false;
  }
  mChildID = child_id;
  UpdateMRMLNodeName();
  return true;
}


const std::string & vtkMRMLROS2Tf2BroadcasterNode::GetChildID(void) const
{
  return mChildID;
}


bool vtkMRMLROS2Tf2BroadcasterNode::IsParentAndChildSet(void)
{
  if (mParentID.empty() || mChildID.empty()) {
    return false;
  }
  else {
    return true;
  }
}


void vtkMRMLROS2Tf2BroadcasterNode::UpdateMRMLNodeName()// Should be a protected method
{
  std::string mMRMLNodeName = "ros2:tf2broadcaster:" + mParentID + "To" + mChildID;
  if (!IsParentAndChildSet()) {
    std::string emptyName = "ros2:tf2broadcaster:empty";
    this->SetName(emptyName.c_str());
    return;
  }
  this->SetName(mMRMLNodeName.c_str());
}


bool vtkMRMLROS2Tf2BroadcasterNode::Broadcast(vtkMRMLTransformNode * message)
{
  // Make sure the parent and child ids are set
  if (!IsParentAndChildSet()) {
    vtkErrorMacro(<< "Child or parent ID not set.");
    return false;
  }

  // Prepare the transform
  geometry_msgs::msg::TransformStamped rosTransform;
  vtkNew<vtkMatrix4x4> matrix;
  message->GetMatrixTransformToParent(matrix); // Note this is an overloaded method (definition below don't need to get matrix from transform)
  vtkSlicerToROS2(matrix, rosTransform, mInternals->mROSNode);
  rosTransform.header.frame_id = mParentID;
  rosTransform.child_frame_id = mChildID;

  // Send the transform
  mInternals->mTfBroadcaster->sendTransform(rosTransform);
  mNumberOfBroadcasts++;
  return true; // just return a bool
}


bool vtkMRMLROS2Tf2BroadcasterNode::Broadcast(vtkMatrix4x4 * message)
{
  // Make sure the parent and child ids are set
  std::string errorMessage;
  if (!IsParentAndChildSet()) {
    vtkErrorMacro(<< "Child or parent ID not set.");
    return false;
  }

  // Prepare the transform
  geometry_msgs::msg::TransformStamped rosTransform;
  vtkSlicerToROS2(message, rosTransform, mInternals->mROSNode);
  rosTransform.header.frame_id = mParentID;
  rosTransform.child_frame_id = mChildID;

  // Send the transform
  mInternals->mTfBroadcaster->sendTransform(rosTransform);
  mNumberOfBroadcasts++;
  return true;
}


void vtkMRMLROS2Tf2BroadcasterNode::ObserveTransformNode(vtkMRMLTransformNode * node)
{
  if (!this->GetScene()->GetNodeByID(node->GetID())) {
    vtkErrorMacro(<< "Transform is not in the scene.");
    return;
  }
  node->AddObserver(vtkMRMLTransformNode::TransformModifiedEvent, this, &vtkMRMLROS2Tf2BroadcasterNode::ObserveTransformCallback);
  this->SetAndObserveNodeReferenceID("ObservedTransform", node->GetID());
}


void vtkMRMLROS2Tf2BroadcasterNode::ObserveTransformCallback(vtkObject * caller, unsigned long,
							     void * vtkNotUsed(callData))
{
  vtkMRMLTransformNode* transformNode = vtkMRMLTransformNode::SafeDownCast(caller);
  if (!transformNode) {
    return;
  }
  else {
     Broadcast(transformNode);
  }
}


void vtkMRMLROS2Tf2BroadcasterNode::WriteXML(ostream & of, int nIndent)
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(mChildID, ChildID);
  vtkMRMLWriteXMLStdStringMacro(mParentID, ParentID);
  vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2Tf2BroadcasterNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(mChildID, ChildID);
  vtkMRMLReadXMLStdStringMacro(mParentID, ParentID);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}
