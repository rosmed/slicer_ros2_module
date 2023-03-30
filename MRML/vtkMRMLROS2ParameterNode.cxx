#include <vtkMRMLROS2ParameterNode.h>

#include <vtkMRMLROS2Utils.h>
#include <vtkMRMLROS2ParameterInternals.h>

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <utility>

vtkStandardNewMacro(vtkMRMLROS2ParameterNode);

vtkMRMLROS2ParameterNode::vtkMRMLROS2ParameterNode()
{
  mInternals = std::make_shared<vtkMRMLROS2ParameterInternals>(this);
}


vtkMRMLROS2ParameterNode::~vtkMRMLROS2ParameterNode()
{
}


void vtkMRMLROS2ParameterNode::PrintSelf(ostream &os, vtkIndent indent)
{
  Superclass::PrintSelf(os, indent);
  // Custom prints
  os << indent << "Monitored Node Name: " << mMonitoredNodeName << "\n";
  os << indent << "MRML Node Name: " << mMRMLNodeName << "\n";
  // os << indent << "ROS type: " << mInternals->GetROSType() << "\n";

  // print contents of mParameterStore
  os << indent << "Monitored Parameters : " << "\n";
  for (const auto &[key, value] : mInternals->mParameterStore) {
    auto param = mInternals->ROS2ParamMsgToParameter(value);
    os << indent << indent << key << ": " << param.value_to_string() << "\n";
  }
}


vtkMRMLNode *vtkMRMLROS2ParameterNode::CreateNodeInstance(void)
{
  return SelfType::New();
}


const char *vtkMRMLROS2ParameterNode::GetNodeTagName(void)
{
  return "ROS2Parameter";
}


bool vtkMRMLROS2ParameterNode::AddToROS2Node(const char * nodeId, const std::string & monitoredNodeName)
{
  mMonitoredNodeName = monitoredNodeName;
  mMRMLNodeName = "ros2:param:" + monitoredNodeName;
  this->SetName(mMRMLNodeName.c_str());

  std::string errorMessage;
  vtkMRMLROS2NodeNode * mrmlROSNodePtr = vtkMRMLROS2::CheckROS2NodeExists(this, nodeId, errorMessage);
  if (!mrmlROSNodePtr) {
    vtkErrorMacro(<< "AddToROS2Node: " << errorMessage);
    return false;
  }

  std::shared_ptr<rclcpp::Node> nodePointer = mrmlROSNodePtr->mInternals->mNodePointer;
  // create a parameter client
  mInternals->mParameterClient = std::make_shared<rclcpp::AsyncParametersClient>(nodePointer, monitoredNodeName);
  // add this parameter node to the ROS node, so that it can be spin in the same thread as the ROS node
  mrmlROSNodePtr->mParameterNodes.push_back(this);
  mrmlROSNodePtr->SetNthNodeReferenceID("parameter",
                                        mrmlROSNodePtr->GetNumberOfNodeReferences("parameter"),
                                        this->GetID());
  this->SetNodeReferenceID("node", nodeId);
  mrmlROSNodePtr->WarnIfNotSpinning("adding parameter client for \"" + monitoredNodeName + "\"");
  mInternals->mMRMLNode = this;
  return true;
}


bool vtkMRMLROS2ParameterNode::RemoveFromROS2Node(const char *nodeId, const std::string &monitoredNodeName)
{
  mMonitoredNodeName = monitoredNodeName;
  mMRMLNodeName = "ros2:param:" + monitoredNodeName;
  std::string errorMessage;
  vtkMRMLROS2NodeNode * rosNodePtr = vtkMRMLROS2::CheckROS2NodeExists(this, nodeId, errorMessage);
  if (!rosNodePtr) {
    vtkErrorMacro(<< "RemoveFromROS2Node: " << errorMessage);
    return false;
  }

  if (!this->IsAddedToROS2Node()) {
    vtkErrorMacro(<< "RemoveFromROS2Node: parameter node is not added to ROS2 node");
    return false;
  }

  // remove the parameter node from the ROS node
  auto it = std::find(rosNodePtr->mParameterNodes.begin(), rosNodePtr->mParameterNodes.end(), this);
  if (it != rosNodePtr->mParameterNodes.end()) {
    rosNodePtr->mParameterNodes.erase(it);
  }

  mInternals->mMRMLNode = nullptr;
  this->SetNodeReferenceID("node", nullptr);
  rosNodePtr->RemoveNthNodeReferenceID("parameter", rosNodePtr->GetNumberOfNodeReferences("parameter"));

  mInternals->mParameterClient.reset();
  return true;
}


// Setting up the parameter event subscriber. If the service is ready, add all monitored parameter values to the parameter server.
bool vtkMRMLROS2ParameterNode::Spin()
{
  // if it is already initialized, return true (was completed in a previous spin)
  if (this->mIsParameterServerReady) {
    return true;
  }

  if (!mInternals->mParameterClient->service_is_ready()) {
    // Print warning macros only once in every 10000 spins
    if (mInternals->serviceNotReadyCounter++ % 10000 == 0) {
      vtkWarningMacro(<< "Spin: parameter service for " << this->mMonitoredNodeName << " is not ready. Please verify if the node is running.");
    }
    return false;
  }

  this->mIsParameterServerReady = true;
  // print that the parameter node is ready for current node
  vtkDebugMacro(<< "Spin: parameter node for " << this->mMonitoredNodeName << " is ready");

  // get a vector of std::string from the map
  std::vector<std::string> parameterNames;
  for (auto &parameter : mInternals->mParameterStore) {
    parameterNames.push_back(parameter.first);
  }

  for (auto &parameter : mInternals->mParameterStore) {
    // use get_parameters to get the parameter value from the parameter server
    auto parameters_future =
      mInternals->mParameterClient->get_parameters({parameter.first},
                                                   std::bind(&vtkMRMLROS2ParameterInternals::GetParametersCallback,
                                                             mInternals, std::placeholders::_1));
  }

  // Setting up the parameter event subscriber.
  mInternals->mParameterEventSubscriber
    = mInternals->mParameterClient
    ->on_parameter_event(std::bind(&vtkMRMLROS2ParameterInternals::ParameterEventCallback, mInternals, std::placeholders::_1));

  return true;
}


// Check if parameter node has been added to Ros2Node
bool vtkMRMLROS2ParameterNode::IsAddedToROS2Node(void) const
{
  return mInternals->mParameterClient != nullptr;
}


// Check if parameter server is ready
bool vtkMRMLROS2ParameterNode::IsMonitoredNodeReady(void) const
{
  //todo: check if this is efficient - potentially add it to spin
  return mInternals->mParameterClient->service_is_ready();
}


bool vtkMRMLROS2ParameterNode::AddParameter(const std::string &parameterName)
{
  if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
    vtkWarningMacro(<< "AddParameter: parameter " << parameterName << " already exists");
    return false;
  }
  mInternals->mParameterStore[parameterName] = rcl_interfaces::msg::Parameter();
  if (!mInternals->mParameterClient->service_is_ready()) {
    vtkWarningMacro(<< "AddParameter: parameter node for " << this->mMonitoredNodeName << " doesn't seem to be available");
    // return false;
  } else {
    auto parameters_future =
      mInternals->mParameterClient->get_parameters({parameterName},
                                                   std::bind(&vtkMRMLROS2ParameterInternals::GetParametersCallback, mInternals, std::placeholders::_1));
  }
  return true;
}


bool vtkMRMLROS2ParameterNode::RemoveParameter(const std::string &parameterName)
{
  if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
    mInternals->mParameterStore.erase(parameterName);
    return true;
  } else {
    vtkWarningMacro(<< "RemoveParameter: parameter " << parameterName << " is not monitored");
    return false;
  }
}


bool vtkMRMLROS2ParameterNode::IsParameterSet(const std::string &parameterName, bool noWarning) const
{
  if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
    return mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
  } else {
    if (!noWarning) {
      vtkWarningMacro(<< "RemoveParameter: parameter " << parameterName << " is not monitored");
    }
    return false;
  }
}


bool vtkMRMLROS2ParameterNode::GetParameterType(const std::string &parameterName, std::string &result)
{
  result = "";
  if (!CheckParameterExistsAndIsSet(parameterName)) {
    return false;
  }
  result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type_name();
  return true;
}


std::string vtkMRMLROS2ParameterNode::GetParameterType(const std::string &parameterName)
{
  std::string result = "";
  GetParameterType(parameterName, result);
  return result;
}


// A method that prints the value of a parameter. If the parameter is not set or if the parameter is not monitored, then it returns false.
std::string vtkMRMLROS2ParameterNode::PrintParameter(const std::string &parameterName)
{
  std::string result = "parameter not found";
  if (!CheckParameterExistsAndIsSet(parameterName)) {
    return result;
  }
  try {
    result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).value_to_string();
  } catch (const std::runtime_error &e) {
    vtkErrorMacro(<< "PrintParameter: parameter " << parameterName << " value cannot be printed: " << e.what());
  }
  return result;
}


bool vtkMRMLROS2ParameterNode::GetParameterAsBool(const std::string &parameterName, bool &result)
{
  result = false;
  if (!CheckParameterExistsAndIsSet(parameterName)) {
    return false;
  }
  try {
    result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_bool();  // if not set add another excep
  } catch (const std::runtime_error &e) {
    vtkErrorMacro(<< "GetParameterAsBool: caught exception for parameter " << parameterName << ": " << e.what());
    return false;
  }
  return true;
}


bool vtkMRMLROS2ParameterNode::GetParameterAsBool(const std::string &parameterName) {
  bool result = false;
  GetParameterAsBool(parameterName, result);
  return result;
}


bool vtkMRMLROS2ParameterNode::GetParameterAsInteger(const std::string &parameterName, int &result)
{
  result = 0;
  if (!CheckParameterExistsAndIsSet(parameterName)) {
    return false;
  }
  try {
    result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_int();
  } catch (const std::runtime_error &e) {
    vtkErrorMacro(<< "GetParameterAsInteger: caught exception for parameter " << parameterName << ": " << e.what());
    return false;
  }
  return true;
}


int vtkMRMLROS2ParameterNode::GetParameterAsInteger(const std::string &parameterName)
{
  int result;
  GetParameterAsInteger(parameterName, result);
  return result;
}

bool vtkMRMLROS2ParameterNode::GetParameterAsDouble(const std::string &parameterName, double &result)
{
  result = 0.0;
  if (!CheckParameterExistsAndIsSet(parameterName)) {
    return false;
  }
  try {
    result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_double();
  } catch (const std::runtime_error &e) {
    vtkErrorMacro(<< "GetParameterAsDouble: caught exception for parameter " << parameterName << ": " << e.what());
    return false;
  }
  return true;
}


double vtkMRMLROS2ParameterNode::GetParameterAsDouble(const std::string &parameterName)
{
  double result;
  GetParameterAsDouble(parameterName, result);
  return result;
}


bool vtkMRMLROS2ParameterNode::GetParameterAsString(const std::string &parameterName, std::string &result)
{
  result = "";
  if (!CheckParameterExistsAndIsSet(parameterName)) {
    return false;
  }
  try {
    result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_string();
  } catch (const std::runtime_error &e) {
    vtkErrorMacro(<< "GetParameterAsString: caught exception for parameter " << parameterName << ": " << e.what());
    return false;
  }
  return true;
}


std::string vtkMRMLROS2ParameterNode::GetParameterAsString(const std::string &parameterName)
{
  std::string result;
  GetParameterAsString(parameterName, result);
  return result;
}


bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfBools(const std::string &parameterName, std::vector<char> &result) {
  result.clear();
  std::vector<bool> tempResult;
  if (!CheckParameterExistsAndIsSet(parameterName)) {
    return false;
  }
  try {
    tempResult = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_bool_array();
  } catch (const std::runtime_error &e) {
    vtkErrorMacro(<< "GetParameterAsVectorOfBools: caught exception for parameter " << parameterName << ": " << e.what());
    return false;
  }
  for (size_t i = 0; i < tempResult.size(); i++) {
    result.push_back(static_cast<bool>(tempResult[i]));
  }
  return true;
}


bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfIntegers(const std::string &parameterName, std::vector<int> &result)
{
  result.clear();
  std::vector<int64_t> tempResult;
  if (!CheckParameterExistsAndIsSet(parameterName)) {
    return false;
  }
  try {
    tempResult = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_integer_array();
  } catch (const std::runtime_error &e) {
    vtkErrorMacro(<< "GetParameterAsVectorOfIntegers: caught exception for parameter " << parameterName << ": " << e.what());
    return false;
  }
  for (auto &i : tempResult) {
    result.push_back(static_cast<int>(i));
  }
  return true;
}


std::vector<int> vtkMRMLROS2ParameterNode::GetParameterAsVectorOfIntegers(const std::string &parameterName)
{
  std::vector<int> result; // todo-address : can use std::vector<int64_t> and change signature of previous method?
  GetParameterAsVectorOfIntegers(parameterName, result);
  return result;
}


bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfDoubles(const std::string &parameterName, std::vector<double> &result)
{
  result.clear();
  if (!CheckParameterExistsAndIsSet(parameterName)) {
    return false;
  }
  try {
    result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_double_array();
  } catch (const std::runtime_error &e) {
    vtkErrorMacro(<< "GetParameterAsVectorOfDoubles: caught exception for parameter " << parameterName << ": " << e.what());
    return false;
  }
  return true;
}


std::vector<double> vtkMRMLROS2ParameterNode::GetParameterAsVectorOfDoubles(const std::string &parameterName)
{
  std::vector<double> result;
  GetParameterAsVectorOfDoubles(parameterName, result);
  return result;
}


bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfStrings(const std::string &parameterName, std::vector<std::string> &result)
{
  result.clear();
  if (!CheckParameterExistsAndIsSet(parameterName)) {
    return false;
  }
  try {
    result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_string_array();
  } catch (const std::runtime_error &e) {
    vtkErrorMacro(<< "GetParameterAsVectorOfStrings: caught exception for parameter " << parameterName << ": " << e.what());
    return false;
  }
  return true;
}


std::vector<std::string> vtkMRMLROS2ParameterNode::GetParameterAsVectorOfStrings(const std::string &parameterName)
{
  std::vector<std::string> result;
  GetParameterAsVectorOfStrings(parameterName, result);
  return result;
}


void vtkMRMLROS2ParameterNode::WriteXML(std::ostream &of, int nIndent)
{
  // add all parameter names from mParameterStore to mMonitoredNodeNames
  for (auto it = mInternals->mParameterStore.begin(); it != mInternals->mParameterStore.end(); ++it) {
    MonitoredParameterNamesCache.push_back(it->first);
  }
  Superclass::WriteXML(of, nIndent);  // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(MRMLNodeName, mMRMLNodeName);
  vtkMRMLWriteXMLStdStringMacro(MonitoredNodeName, mMonitoredNodeName);
  vtkMRMLWriteXMLStdStringVectorMacro(monitoredParameterNames, MonitoredParameterNamesCache, std::vector);
  vtkMRMLWriteXMLEndMacro();
  // clear mMonitoredNodeNames
  MonitoredParameterNamesCache.clear();
}


void vtkMRMLROS2ParameterNode::ReadXMLAttributes(const char **atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts);  // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(nodeName, mMRMLNodeName);
  vtkMRMLReadXMLStdStringMacro(MonitoredNodeName, mMonitoredNodeName);
  vtkMRMLReadXMLStdStringVectorMacro(monitoredParameterNames, MonitoredParameterNamesCache, std::vector);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
  // add an empty parameter msg corresponding to each monitored node name to mParameterStore
  for (const auto & parameterName : MonitoredParameterNamesCache) {
    mInternals->mParameterStore[parameterName] = mInternals->ROS2ParamToParameterMsg(rclcpp::Parameter(parameterName));
  }
}


void vtkMRMLROS2ParameterNode::UpdateScene(vtkMRMLScene *scene)
{
  Superclass::UpdateScene(scene);
  int nbNodeRefs = this->GetNumberOfNodeReferences("node");
  if (nbNodeRefs == 0) {
    // assigned to the default ROS node
    auto defaultNode = scene->GetFirstNodeByName("ros2:node:slicer");
    if(!defaultNode){
      vtkErrorMacro(<< "UpdateScene: default ros2 node unavailable. Unable to set reference for Parameter \"" << GetName() << "\"");
      return;
    }
    this->AddToROS2Node(defaultNode->GetID(), mMonitoredNodeName);
  } else if (nbNodeRefs == 1) {
    this->AddToROS2Node(this->GetNthNodeReference("node", 0)->GetID(), mMonitoredNodeName);
  } else {
    vtkErrorMacro(<< "UpdateScene: more than one ROS2 node reference defined for Parameter \"" << GetName() << "\"");
  }
}


/* Custom Setter for the vector ParameterNamesList */
void vtkMRMLROS2ParameterNode::SetMonitoredParameterNamesCache(const std::vector<std::string> &monitoredParameterNames)
{
  this->MonitoredParameterNamesCache.clear();
  // iterate through the vector and add each parameter name to the list
  for (const auto parameter : monitoredParameterNames) {
    this->MonitoredParameterNamesCache.push_back(parameter);
  }
  this->Modified();
  // this->InvokeCustomModifiedEvent(vtkMRMLROS2ParameterNode::InputDataModifiedEvent);
}


/* Custom Getter for the vector ParameterNamesList */
std::vector<std::string> vtkMRMLROS2ParameterNode::GetMonitoredParameterNamesCache()
{
  std::vector<std::string> monitoredParameterNames;
  for (const auto parameter : this->MonitoredParameterNamesCache) {
    monitoredParameterNames.push_back(parameter);
  }
  return monitoredParameterNames;
}


bool vtkMRMLROS2ParameterNode::CheckParameterExistsAndIsSet(const std::string &parameterName) const
{
  if (mInternals->mParameterStore.find(parameterName) == mInternals->mParameterStore.end()) {
    vtkErrorMacro(<< "CheckParameterExistsAndIsSet: parameter " << parameterName << " does not exist");
    return false;
  }
  // if monitored but not set
  if (mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    vtkErrorMacro(<< "CheckParameterExistsAndIsSet: parameter " << parameterName << " value is not set");
    return false;
  }
  return true;
}
