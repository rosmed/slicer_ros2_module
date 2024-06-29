#include <vtkMRMLROS2ServiceClientNode.h>

#include <vtkMRMLROS2ServiceClientInternals.h>


void vtkMRMLROS2ServiceClientNode::PrintSelf(std::ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
  os << indent << "Topic: " << mTopic << "\n";
  os << indent << "ROS type: " << mInternals->GetROSType() << "\n";
  os << indent << "Slicer type Input: " << mInternals->GetSlicerTypeIn() << "\n"; // This is scrambled
  os << indent << "Slicer type Output: " << mInternals->GetSlicerTypeOut() << "\n"; // This is scrambled
  os << indent << "Number of calls: " << mNumberOfCalls << "\n";
  os << indent << "Number of messages sent:" << mNumberOfRequestsSent << "\n";
}


bool vtkMRMLROS2ServiceClientNode::AddToROS2Node(const char * nodeId,
					     const std::string & topic)
{
  mTopic = topic;
  mMRMLNodeName = "ros2:srv:client:" + topic;
  this->SetName(mMRMLNodeName.c_str());
  std::string errorMessage;
  if (!mInternals->AddToROS2Node(this, nodeId, topic, errorMessage)) {
    vtkErrorMacro(<< "AddToROS2Node: " << errorMessage);
    return false;
  }
  return true;
}


bool vtkMRMLROS2ServiceClientNode::RemoveFromROS2Node(const char * nodeId,
						  const std::string & topic)
{
  if (!mInternals->IsAddedToROS2Node()) {
    vtkErrorMacro(<< "RemoveFromROS2Node: publisher MRML node for topic \"" << mTopic << "\" is not added to the ROS node");
    return false;
  }
  std::string errorMessage;
  if (!mInternals->RemoveFromROS2Node(this, nodeId, topic, errorMessage)) {
    vtkErrorMacro(<< "RemoveFromROS2Node: " << errorMessage);
    return false;
  }
  std::cerr<<"RemoveFromROS2Node: topic: "<<topic<<std::endl;
  return true;
}


bool vtkMRMLROS2ServiceClientNode::IsAddedToROS2Node(void) const
{
  return mInternals->IsAddedToROS2Node();
}


const char * vtkMRMLROS2ServiceClientNode::GetROSType(void) const
{
  return mInternals->GetROSType();
}


const char * vtkMRMLROS2ServiceClientNode::GetSlicerTypeIn(void) const
{
  return mInternals->GetSlicerTypeIn();
}

const char * vtkMRMLROS2ServiceClientNode::GetSlicerTypeOut(void) const
{
  return mInternals->GetSlicerTypeOut();
}


void vtkMRMLROS2ServiceClientNode::WriteXML(ostream& of, int nIndent)
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(topicName, Topic);
  vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2ServiceClientNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(topicName, Topic);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}


void vtkMRMLROS2ServiceClientNode::UpdateScene(vtkMRMLScene *scene)
{
  Superclass::UpdateScene(scene);
  if (!IsAddedToROS2Node()) {
    int nbNodeRefs = this->GetNumberOfNodeReferences("node");
    if (nbNodeRefs == 0) {
      // assigned to the default ROS node
      auto defaultNode = scene->GetFirstNodeByName("ros2:node:slicer");
      if(!defaultNode){
        vtkErrorMacro(<< "UpdateScene: default ros2 node unavailable. Unable to set reference for publisher \"" << GetName() << "\"");
        return;
      }
      this->AddToROS2Node(defaultNode->GetID(), mTopic);
    } else if (nbNodeRefs == 1) {
      this->AddToROS2Node(this->GetNthNodeReference("node", 0)->GetID(), mTopic);
    } else {
      vtkErrorMacro(<< "UpdateScene: more than one ROS2 node reference defined for publisher \"" << GetName() << "\"");
    }
  }
}
