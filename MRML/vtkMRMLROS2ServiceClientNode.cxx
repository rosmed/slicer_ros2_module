#include <vtkMRMLROS2ServiceClientNode.h>

#include <vtkMRMLROS2ServiceClientInternals.h>


void vtkMRMLROS2ServiceClientNode::PrintSelf(std::ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
  os << indent << "Service: " << mService << "\n";
  os << indent << "ROS type: " << mInternals->GetROSType() << "\n";
  os << indent << "Slicer type Input: " << mInternals->GetSlicerTypeIn() << "\n"; // This is scrambled
  os << indent << "Slicer type Output: " << mInternals->GetSlicerTypeOut() << "\n"; // This is scrambled
  os << indent << "Number of calls: " << mNumberOfCalls << "\n";
  os << indent << "Number of messages sent:" << mNumberOfRequestsSent << "\n";
}


bool vtkMRMLROS2ServiceClientNode::AddToROS2Node(const char * nodeId,
					         const std::string & service)
{
  mService = service;
  mMRMLNodeName = "ros2:srv:client:" + service;
  this->SetName(mMRMLNodeName.c_str());
  std::string errorMessage;
  if (!mInternals->AddToROS2Node(this, nodeId, service, errorMessage)) {
    vtkErrorMacro(<< "AddToROS2Node: " << errorMessage);
    return false;
  }
  return true;
}


bool vtkMRMLROS2ServiceClientNode::RemoveFromROS2Node(const char * nodeId,
						      const std::string & service)
{
  if (!mInternals->IsAddedToROS2Node()) {
    vtkErrorMacro(<< "RemoveFromROS2Node: publisher MRML node for service \"" << mService << "\" is not added to the ROS node");
    return false;
  }
  std::string errorMessage;
  if (!mInternals->RemoveFromROS2Node(this, nodeId, service, errorMessage)) {
    vtkErrorMacro(<< "RemoveFromROS2Node: " << errorMessage);
    return false;
  }
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
  vtkMRMLWriteXMLStdStringMacro(serviceName, Service);
  vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2ServiceClientNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(serviceName, Service);
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
      this->AddToROS2Node(defaultNode->GetID(), mService);
    } else if (nbNodeRefs == 1) {
      this->AddToROS2Node(this->GetNthNodeReference("node", 0)->GetID(), mService);
    } else {
      vtkErrorMacro(<< "UpdateScene: more than one ROS2 node reference defined for publisher \"" << GetName() << "\"");
    }
  }
}
