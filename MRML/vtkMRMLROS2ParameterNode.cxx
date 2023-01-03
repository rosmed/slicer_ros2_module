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
    vtkWarningMacro(<< warningMessage);
    return false;
  }
  return true;
 }

 bool vtkMRMLROS2ParameterNode::RemoveParameter(const std::string &nodeName, const std::string &parameterName) {
  std::string warningMessage;
  if (!mInternals->RemoveParameter(nodeName, parameterName, warningMessage)) {
    vtkWarningMacro(<< warningMessage);
    return false;
  }
  return true;
 }

std::string vtkMRMLROS2ParameterNode::GetParameterType(const ParameterKey & key, std::string & result) {
  std::string warningMessage;
  std::string parameterType = mInternals->GetParameterType(key, result, warningMessage);
  if (parameterType.empty()) {
    vtkWarningMacro(<< warningMessage);
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
bool vtkMRMLROS2ParameterNode::GetParameterAsBool(const ParameterKey & key, bool & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsBool(key, result, errorMessage)) {
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

bool vtkMRMLROS2ParameterNode::GetParameterAsDouble(const ParameterKey & key, double & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsDouble(key, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}

bool vtkMRMLROS2ParameterNode::GetParameterAsString(const ParameterKey & key, std::string & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsString(key, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}

bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfBools(const ParameterKey & key, std::vector<bool> & result)
{
  // std::string errorMessage;
  // if (!mInternals->GetParameterAsVectorOfBools(key, result, errorMessage)) {
  //   vtkErrorMacro(<< errorMessage);
  //   return false;
  // }
  return true;
}

bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfIntegers(const ParameterKey & key, std::vector<int64_t> & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsVectorOfIntegers(key, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}

bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfDoubles(const ParameterKey & key, std::vector<double> & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsVectorOfDoubles(key, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}


bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfStrings(const ParameterKey & key, std::vector<std::string> & result)
{
  std::string errorMessage;
  if (!mInternals->GetParameterAsVectorOfStrings(key, result, errorMessage)) {
    vtkErrorMacro(<< errorMessage);
    return false;
  }
  return true;
}

// for debugging only - will be removed
void vtkMRMLROS2ParameterNode::listTrackedParameters(){
  mInternals->listTrackedParameters();
}

std::vector<std::string> vtkMRMLROS2ParameterNode::GetTrackedNodeList(){
  return mInternals->GetTrackedNodeList();
}

std::vector<vtkMRMLROS2ParameterNode::ParameterKey>  vtkMRMLROS2ParameterNode::GetTrackedNodesAndParametersList(){
  return mInternals->GetTrackedNodesAndParametersList();
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
									

