#include <vtkMRMLROS2PublisherCollisionObjectNode.h>
#include <vtkMRMLROS2PublisherInternals.h>
#include <vtkSlicerToROS2.h>
#include <vtkMRMLModelNode.h>
#include <moveit_msgs/msg/collision_object.hpp>

vtkStandardNewMacro(vtkMRMLROS2PublisherCollisionObjectNode);

typedef vtkMRMLROS2PublisherNativeInternals<vtkMRMLModelNode*, moveit_msgs::msg::CollisionObject>
vtkMRMLROS2PublisherCollisionObjectInternals;

vtkMRMLROS2PublisherCollisionObjectNode::vtkMRMLROS2PublisherCollisionObjectNode()
{
  this->AddNodeReferenceRole("source");
  mInternals = new vtkMRMLROS2PublisherCollisionObjectInternals(this);
}

vtkMRMLROS2PublisherCollisionObjectNode::~vtkMRMLROS2PublisherCollisionObjectNode()
{
  delete mInternals;
}

vtkMRMLNode * vtkMRMLROS2PublisherCollisionObjectNode::CreateNodeInstance(void)
{
  return SelfType::New();
}

const char * vtkMRMLROS2PublisherCollisionObjectNode::GetNodeTagName(void)
{
  return "ROS2PublisherCollisionObject";
}

void vtkMRMLROS2PublisherCollisionObjectNode::SetSourceNodeID(const char* sourceNodeID)
{
  this->SetAndObserveNodeReferenceID("source", sourceNodeID);
}

const char* vtkMRMLROS2PublisherCollisionObjectNode::GetSourceNodeID()
{
  return this->GetNodeReferenceID("source");
}

vtkMRMLModelNode* vtkMRMLROS2PublisherCollisionObjectNode::GetSourceNode()
{
  return vtkMRMLModelNode::SafeDownCast(this->GetNodeReference("source"));
}

size_t vtkMRMLROS2PublisherCollisionObjectNode::Publish()
{
  return this->Publish(this->GetSourceNode());
}

size_t vtkMRMLROS2PublisherCollisionObjectNode::Publish(vtkMRMLModelNode* modelNode)
{
  if (!modelNode || !this->IsAddedToROS2Node()) {
    return 0;
  }

  moveit_msgs::msg::CollisionObject msg;
  vtkSlicerToROS2(modelNode, msg, mInternals->GetROSNode());
  msg.header.frame_id = this->FrameId;

  mNumberOfCalls++;
  const auto justSent = (reinterpret_cast<vtkMRMLROS2PublisherCollisionObjectInternals *>(mInternals))->PublishDirect(msg);
  mNumberOfMessagesSent += justSent;
  return justSent;
}
