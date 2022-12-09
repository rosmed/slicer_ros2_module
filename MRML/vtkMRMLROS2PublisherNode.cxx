#include <vtkMRMLROS2PublisherNode.h>

#include <vtkMRMLROS2PublisherInternals.h>

vtkMRMLROS2PublisherNode::vtkMRMLROS2PublisherNode()
{
}

vtkMRMLROS2PublisherNode::~vtkMRMLROS2PublisherNode()
{
}

void vtkMRMLROS2PublisherNode::PrintSelf(std::ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
  os << indent << "Topic: " << mTopic << "\n";
  os << indent << "ROS type: " << mInternals->GetROSType() << "\n";
  os << indent << "Slicer type: " << mInternals->GetSlicerType() << "\n"; // This is scrambled
  os << indent << "Number of calls: " << mNumberOfCalls << "\n";
  os << indent << "Number of messages sent:" << mNumberOfMessagesSent << "\n";
}

bool vtkMRMLROS2PublisherNode::AddToROS2Node(const char * nodeId,
					     const std::string & topic)
{
  mTopic = topic;
  mMRMLNodeName = "ros2:pub:" + topic;
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();

  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, publisher MRML node for topic \"" << topic << "\" needs to be added to the scene first");
    return false;
  }
  std::string errorMessage;
  if (!mInternals->AddToROS2Node(scene, nodeId, topic, errorMessage)) {
    vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
    return false;
  }
  return true;
}

bool vtkMRMLROS2PublisherNode::IsAddedToROS2Node(void) const
{
  return mInternals->IsAddedToROS2Node();
}

const char * vtkMRMLROS2PublisherNode::GetROSType(void) const
{
  return mInternals->GetROSType();
}

const char * vtkMRMLROS2PublisherNode::GetSlicerType(void) const
{
  return mInternals->GetSlicerType();
}

void vtkMRMLROS2PublisherNode::WriteXML(ostream& of, int nIndent)
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  // vtkIndent indent(nIndent);
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(topicName, Topic);
  vtkMRMLWriteXMLEndMacro();
}

void vtkMRMLROS2PublisherNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(topicName, Topic);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}

void vtkMRMLROS2PublisherNode::UpdateScene(vtkMRMLScene *scene)
{
  Superclass::UpdateScene(scene);
  if (!IsAddedToROS2Node()) {
    int nbNodeRefs = this->GetNumberOfNodeReferences("node");
    if (nbNodeRefs != 1) {
      vtkErrorMacro(<< "No ROS2 node reference defined for publisher \"" << GetName() << "\"");
    } else {
      this->AddToROS2Node(this->GetNthNodeReference("node", 0)->GetID(),
			  mTopic);
    }
  }
}
