#include <vtkMRMLROS2Tf2BroadcasterNode.h>
#include <vtkMRMLROS2Tf2BroadcasterInternals.h>
#include <vtkSlicerToROS2.h>
#include <vtkObject.h>
#include <vtkMRMLTransformNode.h>
#include <vtkEventBroker.h>

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
  if (!mInternals->AddToROS2Node(this, scene, nodeId, errorMessage)) {
    vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
    return false;
  }
  return true;
}

void vtkMRMLROS2Tf2BroadcasterNode::SetParentID(const std::string & parent_id)
{
  mParentID = parent_id;
  UpdateMRMLNodeName();
}

std::string vtkMRMLROS2Tf2BroadcasterNode::GetParentID()
{
  return mParentID;
}

void vtkMRMLROS2Tf2BroadcasterNode::SetChildID(const std::string & child_id)
{
  mChildID = child_id;
  UpdateMRMLNodeName();
}

std::string vtkMRMLROS2Tf2BroadcasterNode::GetChildID()
{
  return mChildID;
}

void vtkMRMLROS2Tf2BroadcasterNode::UpdateMRMLNodeName()
{
  // Might not be the best idea - ask Anton
  std::string mMRMLNodeName = "ros2:tf2broadcaster:" + mParentID + "To" + mChildID;
  if (mParentID.empty() || mChildID.empty()){
    std::string mMRMLNodeName = "ros2:tf2broadcaster:empty";
  }
  this->SetName(mMRMLNodeName.c_str());
}

size_t vtkMRMLROS2Tf2BroadcasterNode::Broadcast(vtkMRMLTransformNode * message)
{
  std::string errorMessage;
  if (mParentID.empty() || mChildID.empty()){
    vtkErrorMacro(<< "Child or parent ID not set.");
    return false;
  }
  if (!mInternals->Broadcast(message, mParentID, mChildID)) {
    vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
    return false;
  }
  mNumberOfBroadcasts++;
  return mNumberOfBroadcasts;
}

size_t vtkMRMLROS2Tf2BroadcasterNode::Broadcast(vtkMatrix4x4 * message)
{
  std::string errorMessage;
  if (mParentID.empty() || mChildID.empty()){
    vtkErrorMacro(<< "Child or parent ID not set.");
    return false;
  }
  if (!mInternals->Broadcast(message, mParentID, mChildID)) {
    vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
    return false;
  }
  mNumberOfBroadcasts++;
  return mNumberOfBroadcasts;
}

void vtkMRMLROS2Tf2BroadcasterNode::ObserveTransformNode(vtkMRMLTransformNode * node )
{
  if (!this->GetScene()->GetNodeByID(node->GetID())){
    vtkErrorMacro(<< "Transform is not in the scene.");
    return;
  }
  node->AddObserver(vtkMRMLTransformNode::TransformModifiedEvent, this, &vtkMRMLROS2Tf2BroadcasterNode::ObserveTransformCallback);
  this->SetAndObserveNodeReferenceID("ObservedTransform", node->GetID());
  // Might be worth investigating this event broker
  // vtkEventBroker *broker = vtkEventBroker::GetInstance();
  // broker->AddObservation(node, vtkMRMLTransformNode::TransformModifiedEvent, this, &vtkMRMLROS2Tf2BroadcasterNode::ProcessMRMLNodesEvents)
}

void vtkMRMLROS2Tf2BroadcasterNode::ObserveTransformCallback( vtkObject* caller, unsigned long event, void* vtkNotUsed(callData))
{
  vtkMRMLTransformNode* transformNode = vtkMRMLTransformNode::SafeDownCast(caller);
  if (!transformNode)
  {
    return;
  }
  else
  {
    std::cerr << "Broadcast transform node." << std::endl; // for debugging
    Broadcast(transformNode);
  }
}

void vtkMRMLROS2Tf2BroadcasterNode::WriteXML( ostream& of, int nIndent )
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(mChildID, ChildID);
  vtkMRMLWriteXMLStdStringMacro(mParentID, ParentID);
  vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2Tf2BroadcasterNode::ReadXMLAttributes( const char** atts )
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(mChildID, ChildID);
  vtkMRMLReadXMLStdStringMacro(mParentID, ParentID);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}
