#include <vtkMRMLROS2ParameterInternals.h>
#include <vtkMRMLROS2ParameterNode.h>

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <utility>

vtkStandardNewMacro(vtkMRMLROS2ParameterNode);

vtkMRMLROS2ParameterNode::vtkMRMLROS2ParameterNode() {
    mInternals = new vtkMRMLROS2ParameterInternals(this);
}

vtkMRMLROS2ParameterNode::~vtkMRMLROS2ParameterNode() {
    delete mInternals;
}

void vtkMRMLROS2ParameterNode::PrintSelf(ostream &os, vtkIndent indent) {
    Superclass::PrintSelf(os, indent);
    // Custom prints
    os << indent << "Tracked Node Name: " << mTrackedNodeName << "\n";
    os << indent << "MRML Node Name: " << mMRMLNodeName << "\n";
    // os << indent << "ROS type: " << mInternals->GetROSType() << "\n";

    // print contents of mParameterStore
    for (const auto &[key, value] : mInternals->mParameterStore) {
        auto param = mInternals->ROS2ParamMsgToParameter(value);
        os << indent << indent << key << ": " << param.value_to_string() << "\n";
    }
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

// Setting up the parameter event subscriber.
bool vtkMRMLROS2ParameterNode::SetupParameterEventSubscriber() {
    if (!mInternals->mParameterClient->service_is_ready()) {
        vtkWarningMacro(<< "Parameter Node for " << this->mTrackedNodeName << " doesnt seem to be available");
        return false;
    }

    this->mIsInitialized = true;
    // print that the parameter node is ready for current node
    vtkDebugMacro(<< "Parameter Node for " << this->mTrackedNodeName << " is ready");
    std::cout << "Parameter Node for " << this->mTrackedNodeName << " is ready" << std::endl;

    // get a vector of std::string from the map
    std::vector<std::string> parameterNames;
    for (auto &parameter : mInternals->mParameterStore) {
        parameterNames.push_back(parameter.first);
    }

    auto parameters_future =
        mInternals->mParameterClient->get_parameters(parameterNames,
                                                     std::bind(&vtkMRMLROS2ParameterInternals::GetParametersCallback,
                                                               mInternals, std::placeholders::_1));

    // Setting up the parameter event subscriber.
    mInternals->mParameterEventSubscriber = mInternals->mParameterClient->on_parameter_event(
        std::bind(&vtkMRMLROS2ParameterInternals::ParameterEventCallback, mInternals, std::placeholders::_1));

    return true;
}

bool vtkMRMLROS2ParameterNode::IsAddedToROS2Node(void) const {
    return mInternals->mParameterEventSubscriber != nullptr;
}

bool vtkMRMLROS2ParameterNode::IsParameterServerReady(void) const {
    return mInternals->mParameterClient->service_is_ready();
}

bool vtkMRMLROS2ParameterNode::AddParameter(const std::string &parameterName) {
    if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
        vtkWarningMacro(<< "Parameter " << parameterName << " already exists");
        return false;
    }
    mInternals->mParameterStore[parameterName] = rcl_interfaces::msg::Parameter();
    if (!mInternals->mParameterClient->service_is_ready()) {
        vtkWarningMacro(<< "Parameter Node for " << this->mTrackedNodeName << " doesnt seem to be available. ");
        // return false;
    } else {
        auto parameters_future =
            mInternals->mParameterClient->get_parameters({parameterName},
                                                         std::bind(&vtkMRMLROS2ParameterInternals::GetParametersCallback, mInternals, std::placeholders::_1));
    }
    return true;
}

bool vtkMRMLROS2ParameterNode::RemoveParameter(const std::string &parameterName) {
    if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
        mInternals->mParameterStore.erase(parameterName);
        return true;
    } else {
        vtkWarningMacro(<< "Parameter " << parameterName << " is not tracked");
        return false;
    }
}

bool vtkMRMLROS2ParameterNode::IsParameterSet(const std::string &parameterName) const {
    if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
        return mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
    } else {
        vtkWarningMacro(<< "Parameter " << parameterName << " is not tracked");
        return false;
    }
}

std::string vtkMRMLROS2ParameterNode::GetParameterType(const std::string &parameterName, std::string &result) {
    if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
        result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type_name();
    } else {
        vtkWarningMacro(<< "Parameter " << parameterName << " is not tracked");
        result = "";  // does not exist
    }
    return result;
}

bool vtkMRMLROS2ParameterNode::PrintParameterValue(const std::string &parameterName, std::string &result) {
    if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
        bool parameterRetrievalStatus = true;

        if (mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
            vtkErrorMacro(<< "Parameter " << parameterName << " value is not set");
            return false;
        }

        try {
            result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).value_to_string();
        } catch (const std::runtime_error &e) {
            vtkErrorMacro(<< "Parameter " << parameterName << " value cannot be printed. " << e.what());
            parameterRetrievalStatus = false;
        }

        return parameterRetrievalStatus;
    }
    vtkErrorMacro(<< "Parameter not tracked");
    return false;
}

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. */
bool vtkMRMLROS2ParameterNode::GetParameterAsBool(const std::string &parameterName, bool &result) {
    if (mInternals->mParameterStore.find(parameterName) == mInternals->mParameterStore.end()) {
        vtkErrorMacro(<< "Parameter " << parameterName << " not tracked");
        return false;
    }
    // if tracked but not set
    if (mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        result = false;
        vtkErrorMacro(<< "Parameter " << parameterName << " value is not set");
        return false;
    }
    // if tracked and set
    try {
        result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_bool();  // if not set add another excep
    } catch (const std::runtime_error &e) {
        result = false;
        vtkErrorMacro(<< "Parameter " << parameterName << " GetParameterAsBool caught exception : " << e.what());
        return false;
    }
    return true;
}

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. */
bool vtkMRMLROS2ParameterNode::GetParameterAsInteger(const std::string &parameterName, int &result) {
    if (mInternals->mParameterStore.find(parameterName) == mInternals->mParameterStore.end()) {
        vtkErrorMacro(<< "Parameter " << parameterName << " not tracked");
        return false;
    }
    // if tracked but not set
    if (mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        result = 0;
        vtkErrorMacro(<< "Parameter " << parameterName << " value is not set");
        return false;
    }
    // if tracked and set
    try {
        result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_int();  // if not set add another excep
    } catch (const std::runtime_error &e) {
        result = 0;
        vtkErrorMacro(<< "Parameter " << parameterName << " GetParameterAsInteger caught exception : " << e.what());
        return false;
    }
    return true;
}

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. */
bool vtkMRMLROS2ParameterNode::GetParameterAsDouble(const std::string &parameterName, double &result) {
    if (mInternals->mParameterStore.find(parameterName) == mInternals->mParameterStore.end()) {
        vtkErrorMacro(<< "Parameter " << parameterName << " not tracked");
        return false;
    }
    // if tracked but not set
    if (mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        result = 0.0;
        vtkErrorMacro(<< "Parameter " << parameterName << " value is not set");
        return false;
    }
    // if tracked and set
    try {
        result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_double();  // if not set add another excep
    } catch (const std::runtime_error &e) {
        result = 0.0;
        vtkErrorMacro(<< "Parameter " << parameterName << " GetParameterAsDouble caught exception : " << e.what());
        return false;
    }
    return true;
}

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. */
bool vtkMRMLROS2ParameterNode::GetParameterAsString(const std::string &parameterName, std::string &result) {
    if (mInternals->mParameterStore.find(parameterName) == mInternals->mParameterStore.end()) {
        vtkErrorMacro(<< "Parameter " << parameterName << " not tracked");
        return false;
    }
    // if tracked but not set
    if (mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        result = "";
        vtkErrorMacro(<< "Parameter " << parameterName << " value is not set");
        return false;
    }
    // if tracked and set
    try {
        result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_string();  // if not set add another excep
    } catch (const std::runtime_error &e) {
        result = "";
        vtkErrorMacro(<< "Parameter " << parameterName << " GetParameterAsString caught exception : " << e.what());
        return false;
    }
    return true;
}

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. */
bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfIntegers(const std::string &parameterName, std::vector<int64_t> &result) {
    if (mInternals->mParameterStore.find(parameterName) == mInternals->mParameterStore.end()) {
        vtkErrorMacro(<< "Parameter " << parameterName << " not tracked");
        return false;
    }
    // if tracked but not set
    if (mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        result.clear();
        vtkErrorMacro(<< "Parameter " << parameterName << " value is not set");
        return false;
    }
    // if tracked and set
    try {
        result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_integer_array();  // if not set add another excep
    } catch (const std::runtime_error &e) {
        result.clear();
        vtkErrorMacro(<< "Parameter " << parameterName << " GetParameterAsVectorOfIntegers caught exception : " << e.what());
        return false;
    }
    return true;
}

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. */
bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfDoubles(const std::string &parameterName, std::vector<double> &result) {
    if (mInternals->mParameterStore.find(parameterName) == mInternals->mParameterStore.end()) {
        vtkErrorMacro(<< "Parameter " << parameterName << " not tracked");
        return false;
    }
    // if tracked but not set
    if (mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        result.clear();
        vtkErrorMacro(<< "Parameter " << parameterName << " value is not set");
        return false;
    }
    // if tracked and set
    try {
        result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_double_array();  // if not set add another excep
    } catch (const std::runtime_error &e) {
        result.clear();
        vtkErrorMacro(<< "Parameter " << parameterName << " GetParameterAsVectorOfDoubles caught exception : " << e.what());
        return false;
    }
    return true;
}

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. */
bool vtkMRMLROS2ParameterNode::GetParameterAsVectorOfStrings(const std::string &parameterName, std::vector<std::string> &result) {
    if (mInternals->mParameterStore.find(parameterName) == mInternals->mParameterStore.end()) {
        vtkErrorMacro(<< "Parameter " << parameterName << " not tracked");
        return false;
    }
    // if tracked but not set
    if (mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        result.clear();
        vtkErrorMacro(<< "Parameter " << parameterName << " value is not set");
        return false;
    }
    // if tracked and set
    try {
        result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).as_string_array();  // if not set add another excep
    } catch (const std::runtime_error &e) {
        result.clear();
        vtkErrorMacro(<< "Parameter " << parameterName << " GetParameterAsVectorOfStrings caught exception : " << e.what());
        return false;
    }
    return true;
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
