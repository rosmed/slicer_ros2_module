
#include <vtkMRMLROS2ParameterNode.h>

#include <vtkMRMLROS2ParameterInternals.h>

void vtkMRMLROS2ParameterNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
  // Custom prints
  os << indent << "Topic: " << mTopic << "\n";
  os << indent << "ROS type: " << mInternals->GetROSType() << "\n";
  os << indent << "Slicer type: " << mInternals->GetSlicerType() << "\n"; // This is scrambled
  os << indent << "Number of messages: " << mNumberOfMessages << "\n";
  os << indent << "Last message:" << mInternals->GetLastMessageYAML() << "\n";
}

bool vtkMRMLROS2ParameterNode::AddToROS2Node(const char * nodeId,
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
  if (!mInternals->AddToROS2Node(scene, nodeId, topic, errorMessage)) {
    vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
    return false;
  }
  return true;
}

bool vtkMRMLROS2ParameterNode::IsAddedToROS2Node(void) const
{
  return mInternals->IsAddedToROS2Node();
}

const char * vtkMRMLROS2ParameterNode::GetROSType(void) const
{
  return mInternals->GetROSType();
}

const char * vtkMRMLROS2ParameterNode::GetSlicerType(void) const
{
  return mInternals->GetSlicerType();
}

std::string vtkMRMLROS2ParameterNode::GetLastMessageYAML(void) const
{
  return mInternals->GetLastMessageYAML();
}

void vtkMRMLROS2ParameterNode::WriteXML(std::ostream& of, int nIndent)
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(topicName, Topic);
  vtkMRMLWriteXMLEndMacro();
}

void vtkMRMLROS2ParameterNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(topicName, Topic);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}

void vtkMRMLROS2ParameterNode::UpdateScene(vtkMRMLScene *scene)
{
  Superclass::UpdateScene(scene);
  int nbNodeRefs = this->GetNumberOfNodeReferences("node");
  if (nbNodeRefs != 1) {
    vtkErrorMacro(<< "No ROS2 node reference defined for subscriber \"" << GetName() << "\"");
  } else {
    this->AddToROS2Node(this->GetNthNodeReference("node", 0)->GetID(),
			mTopic);
  }
}
