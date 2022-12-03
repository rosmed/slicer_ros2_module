
#include <vtkMRMLROS2SubscriberNode.h>

#include <vtkMRMLROS2SubscriberInternals.h>

vtkMRMLROS2SubscriberNode::vtkMRMLROS2SubscriberNode()
{
}

vtkMRMLROS2SubscriberNode::~vtkMRMLROS2SubscriberNode()
{
}

bool vtkMRMLROS2SubscriberNode::AddToROS2Node(const char * nodeId,
					      const std::string & topic)
{
  mTopic = topic;
  mMRMLNodeName = "ros2:sub:" + topic;
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkWarningMacro(<< "AddToROS2Node, subscriber MRML node for topic \"" << topic << "\" needs to be added to the scene first");
    return false;
  }
  std::string errorMessage;
  if (mInternals->AddToROS2Node(scene, nodeId, topic, errorMessage)) {
    return true;
  }
  vtkWarningMacro(<< "AddToROS2Node, looking for ROS2 node: " << errorMessage);
  return false;
}

const char * vtkMRMLROS2SubscriberNode::GetTopic(void) const
{
  return mTopic.c_str();
}

const char * vtkMRMLROS2SubscriberNode::GetROSType(void) const
{
  return mInternals->GetROSType();
}

const char * vtkMRMLROS2SubscriberNode::GetSlicerType(void) const
{
  return mInternals->GetSlicerType();
}

size_t vtkMRMLROS2SubscriberNode::GetNumberOfMessages(void) const
{
  return mNumberOfMessages;
}

std::string vtkMRMLROS2SubscriberNode::GetLastMessageYAML(void) const
{
  return mInternals->GetLastMessageYAML();
}
