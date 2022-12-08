
#include <vtkMRMLROS2SubscriberNode.h>

#include <vtkMRMLROS2SubscriberInternals.h>

vtkMRMLROS2SubscriberNode::vtkMRMLROS2SubscriberNode()
{
}

vtkMRMLROS2SubscriberNode::~vtkMRMLROS2SubscriberNode()
{
}

void vtkMRMLROS2SubscriberNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
  // Custom prints
  os << indent << "Topic: " << mTopic << "\n";
  os << indent << "ROS type: " << mInternals->GetROSType() << "\n";
  os << indent << "Slicer type: " << mInternals->GetSlicerType() << "\n"; // This is scrambled
  os << indent << "Number of Messages: " << mNumberOfMessages << "\n"; 
  os << indent << "Last message:" << mInternals->GetLastMessageYAML() << "\n"; 
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

  parentNodeID.assign(nodeId);

  std::string errorMessage;
  if (mInternals->AddToROS2Node(scene, nodeId, topic, errorMessage)) {
    return true;
  }
  else{
    vtkErrorMacro(<< "Subscriber by that name is already in the scene.");
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


void vtkMRMLROS2SubscriberNode::WriteXML( ostream& of, int nIndent )
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkIndent indent(nIndent);

  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(topicName, mTopic);
  vtkMRMLWriteXMLStdStringMacro(parentNodeID, parentNodeID);
  vtkMRMLWriteXMLEndMacro();
}

//------------------------------------------------------------------------------
void vtkMRMLROS2SubscriberNode::ReadXMLAttributes( const char** atts )
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(topicName, mTopic);
  vtkMRMLReadXMLStdStringMacro(parentNodeID, parentNodeID);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}

void vtkMRMLROS2SubscriberNode::UpdateScene(vtkMRMLScene *scene)
{
    Superclass::UpdateScene(scene);
    this->AddToROS2Node(parentNodeID.c_str(),mTopic);
    std::cerr << "Subscriber updated" << std::endl;
}
