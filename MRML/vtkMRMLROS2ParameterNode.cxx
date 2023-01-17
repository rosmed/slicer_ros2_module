#include <vtkMRMLROS2ParameterInternals.h>
#include <vtkMRMLROS2ParameterNode.h>

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <utility>

vtkStandardNewMacro(vtkMRMLROS2ParameterNode);

void vtkMRMLROS2ParameterNode::PrintSelf(ostream &os, vtkIndent indent) {
  Superclass::PrintSelf(os, indent);
  // Custom prints
  // os << indent << "Topic: " << mTopic << "\n";
  // os << indent << "ROS type: " << mInternals->GetROSType() << "\n";
  // os << indent << "Slicer type: " << mInternals->GetSlicerType() << "\n"; // This is scrambled
  // os << indent << "Number of messages: " << mNumberOfMessages << "\n";
  // os << indent << "Last message:" << mInternals->GetLastMessageYAML() << "\n";
}

vtkMRMLNode *vtkMRMLROS2ParameterNode::CreateNodeInstance(void) {
  return SelfType::New();
}

const char *vtkMRMLROS2ParameterNode::GetNodeTagName(void) {
  return "ROS2Parameter";
}

bool vtkMRMLROS2ParameterNode::AddToROS2Node(const char *nodeId, const std::string &trackedNodeName) {
  this->SetName(mMRMLNodeName.c_str());
  mTrackedNodeName = trackedNodeName;
  mMRMLNodeName = "ros2:param:" + trackedNodeName;
  vtkMRMLScene *scene = this->GetScene();
  if (!this->GetScene()) {
    vtkWarningMacro(<< "AddToROS2Node, parameter MRML node needs to be added to the scene first");
    return false;
  }

  vtkMRMLNode *rosNodeBasePtr = scene->GetNodeByID(nodeId);

  std::string errorMessage;
  if (!rosNodeBasePtr) {
    errorMessage = "AddToROS2Node, ROS node with id " + std::string(nodeId) + " not found in the scene";
    vtkWarningMacro(<< errorMessage);
    return false;
  }

  vtkMRMLROS2NODENode *rosNodePtr = dynamic_cast<vtkMRMLROS2NODENode *>(rosNodeBasePtr);

  if (!rosNodePtr) {
    errorMessage = "AddToROS2Node, ROS node with id " + std::string(nodeId) + " is not a vtkMRMLROS2NODENode";
    vtkWarningMacro(<< errorMessage);
    return false;
  }

  std::shared_ptr<rclcpp::Node> nodePointer = rosNodePtr->mInternals->mNodePointer;

  mInternals->mParameterClient = std::make_shared<rclcpp::AsyncParametersClient>(nodePointer, trackedNodeName);

  rosNodePtr->mParameterNodes.push_back(this);

  rosNodePtr->SetNthNodeReferenceID("parameter", rosNodePtr->GetNumberOfNodeReferences("parameter"),
                                    this->GetID());

  this->SetNodeReferenceID("node", nodeId);

  return true;
}

bool vtkMRMLROS2ParameterNode::SetupParameterEventSubscriber() {
  if (!mInternals->mParameterClient->service_is_ready()) {
    vtkWarningMacro(<< "Parameter Node for " << this->mTrackedNodeName << " doesnt seem to be available");
    return false;
  }

  this->mIsInitialized = true;
  std::cerr << "service is ready" << std::endl;

  // get a vector of std::string from the map

  std::vector<std::string> parameterNames;
  for (auto &parameter : mInternals->mParameterStore) {
    parameterNames.push_back(parameter.first);
  }

  auto parameters_future =
      mInternals->mParameterClient->get_parameters(parameterNames,
                                                   std::bind(&vtkMRMLROS2ParameterInternals::GetParametersCallback,
                                                             mInternals, std::placeholders::_1));

  mInternals->mParameterEventSubscriber = mInternals->mParameterClient->on_parameter_event(
      std::bind(&vtkMRMLROS2ParameterInternals::ParameterEventCallback, mInternals, std::placeholders::_1));

  return true;
}

bool vtkMRMLROS2ParameterNode::IsAddedToROS2Node(void) const {
  return mInternals->mParameterEventSubscriber != nullptr;
}

bool vtkMRMLROS2ParameterNode::AddParameter(const std::string &parameterName) {
  if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
    vtkWarningMacro(<< "Parameter already tracked");
    return false;
  }
  mInternals->mParameterStore[parameterName] = rcl_interfaces::msg::Parameter();
  if (!mInternals->mParameterClient->service_is_ready()) {
    vtkWarningMacro(<< "Parameter Node for " << this->mTrackedNodeName << " doesnt seem to be available. ");
    return vtkMRMLROS2Tf2BufferNode;
  } else {
    mInternals->mParameterStore[parameterName] = rcl_interfaces::msg::Parameter();
    auto parameters_future =
        mInternals->mParameterClient->get_parameters({parameterName},
                                                     std::bind(&vtkMRMLROS2ParameterInternals::GetParameterCallback, mInternals, std::placeholders::_1));
  }
  return true;
}

bool vtkMRMLROS2ParameterNode::RemoveParameter(const std::string &parameterName) {
  if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
    mInternals->mParameterStore.erase(parameterName);
    return true;
  } else {
    vtkWarningMacro(<< "Parameter not tracked");
    return false;
  }
}

bool vtkMRMLROS2ParameterNode::IsParameterSet(const std::string &parameterName) const {
  if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
    return mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
  } else {
    vtkWarningMacro(<< "Parameter not tracked");
    return false;
  }
}

std::string vtkMRMLROS2ParameterNode::GetParameterType(const std::string &parameterName, std::string &result) {
  std::string warningMessage;
  if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
    result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type_name();
  } else {
    warningMessage = "parameter : " + parameterName + " is not tracked";
    vtkWarningMacro(<< warningMessage);
    result = "";  // does not exist
  }
  return result;
}

bool vtkMRMLROS2ParameterNode::PrintParameterValue(const std::string &parameterName, std::string &result) {
  std::string errorMessage;
  if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
    bool parameterRetrievalStatus = true;

    if (mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      errorMessage = "Parameter value not set";
      vtkErrorMacro(<< errorMessage);
      return false;
    }

    try {
      result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).value_to_string();
    } catch (const std::runtime_error &e) {
      errorMessage = "PrintParameterValue caught exception :";
      errorMessage.append(e.what());
      parameterRetrievalStatus = false;
      vtkErrorMacro(<< errorMessage);
    }

    return parameterRetrievalStatus;
  }
  vtkErrorMacro(<< "Parameter not tracked");
  return false;
}

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. */
bool vtkMRMLROS2ParameterNode::GetParameterAsBool(const std::string &parameterName, bool &result) {
  std::string errorMessage;
  if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
    bool parameterRetrievalStatus = true;
    try {
      result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_bool();  // if not set add another excep
    } catch (const std::runtime_error &e) {
      errorMessage = "GetParameterAsBoolean caught exception :";
      errorMessage.append(e.what());
      parameterRetrievalStatus = false;
      vtkErrorMacro(<< errorMessage);
    }
    return parameterRetrievalStatus;
  }
  errorMessage = "Parameter not tracked";
  vtkErrorMacro(<< errorMessage);
  return false;
}

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. */
bool vtkMRMLROS2ParameterNode::GetParameterAsInteger(const std::string &parameterName, int &result) {
  std::string errorMessage;
  if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
    bool parameterRetrievalStatus = true;
    try {
      result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_int();
    } catch (const std::runtime_error &e) {
      errorMessage = "GetParameterAsInteger caught exception :";
      errorMessage.append(e.what());
      parameterRetrievalStatus = false;
      vtkErrorMacro(<< errorMessage);
    }
    return parameterRetrievalStatus;
  }
  errorMessage = "Parameter not tracked";
  vtkErrorMacro(<< errorMessage);
  return false;
}

// for debugging only - will be removed
void vtkMRMLROS2ParameterNode::listTrackedParameters() {
  for (const auto &[key, value] : mInternals->mParameterStore) {
    auto param = mInternals->ROS2ParamMsgToParameter(value);

    std::cerr << "-->" << key << ", " << param.value_to_string() << std::endl;
  }
}

std::vector<std::string> vtkMRMLROS2ParameterNode::GetTrackedParametersList() {
  std::vector<std::string> ParametersList;
  for (const auto &[key, value] : mInternals->mParameterStore) {
    ParametersList.push_back(key);
  }
  return ParametersList;
}

void vtkMRMLROS2ParameterNode::WriteXML(std::ostream &of, int nIndent) {
  Superclass::WriteXML(of, nIndent);  // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(nodeName, mMRMLNodeName);
  vtkMRMLWriteXMLEndMacro();
}

void vtkMRMLROS2ParameterNode::ReadXMLAttributes(const char **atts) {
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts);  // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(nodeName, mMRMLNodeName);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}

// TODO : Handle references??
void vtkMRMLROS2ParameterNode::UpdateScene(vtkMRMLScene *scene) {
  Superclass::UpdateScene(scene);
  int nbNodeRefs = this->GetNumberOfNodeReferences("node");
  if (nbNodeRefs != 1) {
    vtkErrorMacro(<< "No ROS2 node reference defined for parameter subscriber \"" << GetName() << "\"");
  } else {
    this->AddToROS2Node(this->GetNthNodeReference("node", 0)->GetID(), mTrackedNodeName);
  }
}

vtkMRMLROS2ParameterNode::vtkMRMLROS2ParameterNode() {
  mInternals = new vtkMRMLROS2ParameterInternals(this);
}

vtkMRMLROS2ParameterNode::~vtkMRMLROS2ParameterNode() {
  delete mInternals;
}
