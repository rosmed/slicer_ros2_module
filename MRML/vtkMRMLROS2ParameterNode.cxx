#include <vtkMRMLROS2ParameterNode.h>
#include <vtkMRMLROS2ParameterInternals.h>

vtkStandardNewMacro(vtkMRMLROS2ParameterNode);

// typedef vtkMRMLROS2ParameterNativeInternals<ros_type, slicer_type>	vtkMRMLROS2ParameterInternals;

void vtkMRMLROS2ParameterNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
  // Custom prints
  // os << indent << "Topic: " << mTopic << "\n";
  // os << indent << "ROS type: " << mInternals->GetROSType() << "\n";
  // os << indent << "Slicer type: " << mInternals->GetSlicerType() << "\n"; // This is scrambled
  // os << indent << "Number of messages: " << mNumberOfMessages << "\n";
  // os << indent << "Last message:" << mInternals->GetLastMessageYAML() << "\n";
}

 vtkMRMLNode * vtkMRMLROS2ParameterNode::CreateNodeInstance(void){									
   return SelfType::New();						
 }									
									
 const char * vtkMRMLROS2ParameterNode::GetNodeTagName(void){									
   return "ROS2Parameter";					
 }	

bool vtkMRMLROS2ParameterNode::AddToROS2Node(const char * nodeId) {
    this->SetName(mMRMLNodeName.c_str());
    vtkMRMLScene * scene = this->GetScene();
    if (!this->GetScene()) {
      vtkWarningMacro(<< "AddToROS2Node, parameter MRML node needs to be added to the scene first");
      return false;
    }
    std::string errorMessage;
    if (!mInternals->AddToROS2Node(scene, nodeId, errorMessage)) {
      vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
      return false;
    }
    return true;
}

bool vtkMRMLROS2ParameterNode::IsAddedToROS2Node(void) const {
  return mInternals->IsAddedToROS2Node();
}

bool vtkMRMLROS2ParameterNode::AddParameter(const std::string &nodeName, const std::string &parameterName) {
  std::string warningMessage;
  if (!mInternals->AddParameter(nodeName, parameterName, warningMessage)) {
    vtkErrorMacro(<< warningMessage);
    return false;
  }
  return true;
 }

std::string vtkMRMLROS2ParameterNode::GetParameterType(const std::string &nodeName, const std::string &parameterName) {
  std::string warningMessage;
  std::string parameterType = mInternals->GetParameterType(nodeName, parameterName, warningMessage);
  if (parameterType.empty()) {
    vtkErrorMacro(<< warningMessage);
  }
  return parameterType;
}

bool vtkMRMLROS2ParameterNode::PrintParameterValue(const ParameterKey & key, std::string & result)
{
  std::string errorMessage;
  if (!mInternals->PrintParameterValue(key, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}

/*! Users should always make sure the key exists and the parameter type is a string with GetParameterType before calling this method. */
bool vtkMRMLROS2ParameterNode::GetParameterAsString(const ParameterKey & key, std::string & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsString(key, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}

bool vtkMRMLROS2ParameterNode::GetParameterAsInteger(const ParameterKey & key, int & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsInteger(key, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}


// for debugging only - will be removed
void vtkMRMLROS2ParameterNode::listTrackedParameters(){
  mInternals->listTrackedParameters();
}


 // void vtkMRMLROS2ParameterNode::WriteXML(std::ostream& of, int nIndent)
// {
//   Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
//   vtkMRMLWriteXMLBeginMacro(of);
//   vtkMRMLWriteXMLStdStringMacro(topicName, Topic);
//   vtkMRMLWriteXMLEndMacro();
// }

// void vtkMRMLROS2ParameterNode::ReadXMLAttributes(const char** atts)
// {
//   int wasModifying = this->StartModify();
//   Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
//   vtkMRMLReadXMLBeginMacro(atts);
//   vtkMRMLReadXMLStdStringMacro(topicName, Topic);
//   vtkMRMLReadXMLEndMacro();
//   this->EndModify(wasModifying);
// }

// void vtkMRMLROS2ParameterNode::UpdateScene(vtkMRMLScene *scene)
// {
//   Superclass::UpdateScene(scene);
//   int nbNodeRefs = this->GetNumberOfNodeReferences("node");
//   if (nbNodeRefs != 1) {
//     vtkErrorMacro(<< "No ROS2 node reference defined for subscriber \"" << GetName() << "\"");
//   } else {
//     this->AddToROS2Node(this->GetNthNodeReference("node", 0)->GetID(),
// 			mTopic);
//   }
// }

 vtkMRMLROS2ParameterNode::vtkMRMLROS2ParameterNode()	{									
   mInternals = new vtkMRMLROS2ParameterInternals(this);	
 }									
									
 vtkMRMLROS2ParameterNode::~vtkMRMLROS2ParameterNode() {									
   delete mInternals;							
 }									
									

