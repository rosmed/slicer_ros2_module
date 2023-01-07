#include <vtkMRMLROS2ParameterNode.h>
#include <vtkMRMLROS2ParameterInternals.h>

vtkStandardNewMacro(vtkMRMLROS2ParameterNode);

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

bool vtkMRMLROS2ParameterNode::AddToROS2Node(const char * nodeId, const std::string & trackedNodeName) {
    this->SetName(mMRMLNodeName.c_str());
    mMRMLNodeName = "ros2:param:" + trackedNodeName;
    vtkMRMLScene * scene = this->GetScene();
    if (!this->GetScene()) {
      vtkWarningMacro(<< "AddToROS2Node, parameter MRML node needs to be added to the scene first");
      return false;
    }
    std::string errorMessage;
    if (!mInternals->AddToROS2Node(scene, nodeId, trackedNodeName, errorMessage)) {
      vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
      return false;
    }
    return true;
}

bool vtkMRMLROS2ParameterNode::IsAddedToROS2Node(void) const {
  return mInternals->IsAddedToROS2Node();
}

bool vtkMRMLROS2ParameterNode::AddParameter(const std::string &parameterName) {
  std::string warningMessage;
  if (!mInternals->AddParameter(parameterName, warningMessage)) {
    vtkWarningMacro(<< warningMessage);
    return false;
  }
  return true;
 }

 bool vtkMRMLROS2ParameterNode::RemoveParameter(const std::string &parameterName) {
  std::string warningMessage;
  if (!mInternals->RemoveParameter(parameterName, warningMessage)) {
    vtkWarningMacro(<< warningMessage);
    return false;
  }
  return true;
 }

std::string vtkMRMLROS2ParameterNode::GetParameterType(const std::string & parameterName, std::string & result) {
  std::string warningMessage;
  std::string parameterType = mInternals->GetParameterType(parameterName, result, warningMessage);
  if (parameterType.empty()) {
    vtkWarningMacro(<< warningMessage);
  }
  return parameterType;
}

bool vtkMRMLROS2ParameterNode::PrintParameterValue(const std::string & parameterName, std::string & result)
{
  std::string errorMessage;
  if (!mInternals->PrintParameterValue(parameterName, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. */
bool vtkMRMLROS2ParameterNode::GetParameterAsBool(const std::string & parameterName, bool & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsBool(parameterName, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}

bool vtkMRMLROS2ParameterNode::GetParameterAsInteger(const std::string & parameterName, int & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsInteger(parameterName, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}

bool vtkMRMLROS2ParameterNode::GetParameterAsDouble(const std::string & parameterName, double & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsDouble(parameterName, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}

bool vtkMRMLROS2ParameterNode::GetParameterAsString(const std::string & parameterName, std::string & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsString(parameterName, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}

// bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfBools(const std::string & parameterName, std::vector<bool> & result)
// {
//   std::string errorMessage;
//   if (!mInternals->GetParameterAsVectorOfBools(parameterName, result, errorMessage)) {
//     vtkErrorMacro(<< errorMessage);
//     return false;
//   }
//   return true;
// }

bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfIntegers(const std::string & parameterName, std::vector<int64_t> & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsVectorOfIntegers(parameterName, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}

bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfDoubles(const std::string & parameterName, std::vector<double> & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsVectorOfDoubles(parameterName, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}


bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfStrings(const std::string & parameterName, std::vector<std::string> & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsVectorOfStrings(parameterName, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}

// for debugging only - will be removed
void vtkMRMLROS2ParameterNode::listTrackedParameters(){
  mInternals->listTrackedParameters();
}

std::vector<std::string>  vtkMRMLROS2ParameterNode::GetTrackedParametersList(){
  return mInternals->GetTrackedParametersList();
}

 void vtkMRMLROS2ParameterNode::WriteXML(std::ostream& of, int nIndent)
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(nodeName, mMRMLNodeName);
  vtkMRMLWriteXMLEndMacro();
}

void vtkMRMLROS2ParameterNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(nodeName, mMRMLNodeName);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}

// TODO : Handle references?? 
void vtkMRMLROS2ParameterNode::UpdateScene(vtkMRMLScene *scene)
{
  Superclass::UpdateScene(scene);
  // int nbNodeRefs = this->GetNumberOfNodeReferences("node");
  // if (nbNodeRefs != 1) {
  //   vtkErrorMacro(<< "No ROS2 node reference defined for parameter subscriber \"" << GetName() << "\"");
  // } else {
  //   this->AddToROS2Node(this->GetNthNodeReference("node", 0)->GetID(),
	// 		mTopic);
  // }
}

 vtkMRMLROS2ParameterNode::vtkMRMLROS2ParameterNode()	{									
   mInternals = new vtkMRMLROS2ParameterInternals(this);	
 }									
									
 vtkMRMLROS2ParameterNode::~vtkMRMLROS2ParameterNode() {									
   delete mInternals;							
 }									
									

