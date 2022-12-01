#include <vtkMRMLROS2PublisherNode.h>

#include <vtkMRMLROS2PublisherInternals.h>

vtkMRMLROS2PublisherNode::vtkMRMLROS2PublisherNode()
{
}

vtkMRMLROS2PublisherNode::~vtkMRMLROS2PublisherNode()
{
}

bool vtkMRMLROS2PublisherNode::AddToROS2Node(const char * nodeId,
					      const std::string & topic)
{
  mTopic = topic;
  mMRMLNodeName = "ros2:sub:" + topic;
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkWarningMacro(<< "AddToROS2Node, Publisher MRML node for topic \"" << topic << "\" needs to be added to the scene first");
    return false;
  }
  std::string errorMessage;
  if (mInternals->AddToROS2Node(scene, nodeId, topic, errorMessage)) {
    return true;
  }
  vtkWarningMacro(<< "AddToROS2Node, looking for ROS2 node: " << errorMessage);
  return false;
}

const char * vtkMRMLROS2PublisherNode::GetTopic(void) const
{
  return mTopic.c_str();
}

const char * vtkMRMLROS2PublisherNode::GetROSType(void) const
{
  return mInternals->GetROSType();
}

const char * vtkMRMLROS2PublisherNode::GetSlicerType(void) const
{
  return mInternals->GetSlicerType();
}

size_t vtkMRMLROS2PublisherNode::GetNumberOfMessages(void) const
{
  return mNumberOfMessages;
}

std::string vtkMRMLROS2PublisherNode::GetLastMessageYAML(void) const
{
  return mInternals->GetLastMessageYAML();
}
