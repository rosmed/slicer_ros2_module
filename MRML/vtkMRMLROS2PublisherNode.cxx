#include <vtkMRMLROS2PublisherNode.h>

#include <vtkMRMLROS2PublisherInternals.h>

vtkMRMLROS2PublisherNode::vtkMRMLROS2PublisherNode()
{
}

vtkMRMLROS2PublisherNode::~vtkMRMLROS2PublisherNode()
{
}

void vtkMRMLROS2PublisherNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
  os << indent << "Topic: " << mTopic << "\n";
  os << indent << "ROS type: " << mInternals->GetROSType() << "\n";
  os << indent << "Slicer type: " << mInternals->GetSlicerType() << "\n"; // This is scrambled
  os << indent << "Number of calls: " << mNumberOfCalls << "\n"; 
  os << indent << "Number of messages sent:" << mNumberOfMessagesSent << "\n"; // This should be replaced with GetLastMessage
}

bool vtkMRMLROS2PublisherNode::AddToROS2Node(const char * nodeId,
					      const std::string & topic)
{
  mTopic = topic;
  mMRMLNodeName = "ros2:pub:" + topic;
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();

  if (!this->GetScene()) {
    vtkWarningMacro(<< "AddToROS2Node, Publisher MRML node for topic \"" << topic << "\" needs to be added to the scene first");
    return false;
  }

  parentNodeID.assign(nodeId);

  std::string errorMessage;
  if (mInternals->AddToROS2Node(scene, nodeId, topic, errorMessage)) {
    return true;
  }
  else{
    vtkWarningMacro(<< "Publisher for this topic: \"" << topic << "\" is already in the scene.");
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

size_t vtkMRMLROS2PublisherNode::GetNumberOfCalls(void) const
{
  return mNumberOfCalls;
}

size_t vtkMRMLROS2PublisherNode::GetNumberOfMessagesSent(const char * nodeId, const std::string & topic)
{
  vtkMRMLScene * scene = this->GetScene();
  return mInternals->GetNumberOfMessagesSent(scene, nodeId, topic);
}

void vtkMRMLROS2PublisherNode::WriteXML( ostream& of, int nIndent )
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkIndent indent(nIndent);

  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(topicName, mTopic);
  vtkMRMLWriteXMLStdStringMacro(parentNodeID, parentNodeID);
  vtkMRMLWriteXMLEndMacro();
}

//------------------------------------------------------------------------------
void vtkMRMLROS2PublisherNode::ReadXMLAttributes( const char** atts )
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(topicName, mTopic);
  vtkMRMLReadXMLStdStringMacro(parentNodeID, parentNodeID);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}

void vtkMRMLROS2PublisherNode::UpdateScene(vtkMRMLScene *scene)
{
    Superclass::UpdateScene(scene);
    this->AddToROS2Node(parentNodeID.c_str(), mTopic);
    std::cerr << "Publisher updated. " << std::endl;
}