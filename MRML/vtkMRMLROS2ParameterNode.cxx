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
    os << indent << "Tracked Parameters : " << "\n";
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
    if (!rosNodeBasePtr) {
        vtkWarningMacro(<< "AddToROS2Node, ROS node with id " + std::string(nodeId) + " not found in the scene");
        return false;
    }

    vtkMRMLROS2NODENode *rosNodePtr = dynamic_cast<vtkMRMLROS2NODENode *>(rosNodeBasePtr);
    if (!rosNodePtr) {
        vtkWarningMacro(<< "AddToROS2Node, ROS node with id " + std::string(nodeId) + " is not a vtkMRMLROS2NODENode");
        return false;
    }

    std::shared_ptr<rclcpp::Node> nodePointer = rosNodePtr->mInternals->mNodePointer;
    // create a parameter client
    mInternals->mParameterClient = std::make_shared<rclcpp::AsyncParametersClient>(nodePointer, trackedNodeName);
    // add this parameter node to the ROS node, so that it can be spin in the same thread as the ROS node
    rosNodePtr->mParameterNodes.push_back(this);
    rosNodePtr->SetNthNodeReferenceID("parameter", rosNodePtr->GetNumberOfNodeReferences("parameter"),
                                      this->GetID());
    this->SetNodeReferenceID("node", nodeId);

    return true;
}

// Setting up the parameter event subscriber. If the service is ready, add all tracked parameter values to the parameter server.
bool vtkMRMLROS2ParameterNode::SetupParameterEventSubscriber() {
    if (!mInternals->mParameterClient->service_is_ready()) {
        // Print warning macros only once in every 10000 spins
        if (mInternals->serviceNotReadyCounter++ % 10000 == 0) {
            vtkWarningMacro(<< "Parameter service for " << this->mTrackedNodeName << " is not ready. Please verify if the node is running.");
        }
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

    for (auto &parameter : mInternals->mParameterStore) {
    // use get_parameters to get the parameter value from the parameter server
        auto parameters_future =
            mInternals->mParameterClient->get_parameters({parameter.first},
                                                         std::bind(&vtkMRMLROS2ParameterInternals::GetParametersCallback,
                                                                   mInternals, std::placeholders::_1));
    }

    // Setting up the parameter event subscriber.
    mInternals->mParameterEventSubscriber = mInternals->mParameterClient->on_parameter_event(
        std::bind(&vtkMRMLROS2ParameterInternals::ParameterEventCallback, mInternals, std::placeholders::_1));

    return true;
}

// Check if parameter node has been added to Ros2Node
bool vtkMRMLROS2ParameterNode::IsAddedToROS2Node(void) const {
    return mInternals->mParameterClient != nullptr;
}

// Check if parameter server is ready 
bool vtkMRMLROS2ParameterNode::IsParameterServerReady(void) const {
    return mInternals->mParameterClient->service_is_ready();
}

bool vtkMRMLROS2ParameterNode::AddParameterForTracking(const std::string &parameterName) {
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

bool vtkMRMLROS2ParameterNode::RemoveParameterFromTracking(const std::string &parameterName) {
    if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
        mInternals->mParameterStore.erase(parameterName);
        return true;
    } else {
        vtkWarningMacro(<< "Parameter " << parameterName << " is not tracked");
        return false;
    }
}

bool vtkMRMLROS2ParameterNode::IsParameterValueSet(const std::string &parameterName) const {
    if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
        return mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
    } else {
        vtkWarningMacro(<< "Parameter " << parameterName << " is not tracked");
        return false;
    }
}

// Method for getting the type of the parameter as string. If the parameter is not set, then it returns an empty string.
std::string vtkMRMLROS2ParameterNode::GetParameterType(const std::string &parameterName, std::string &result) {
    if (mInternals->mParameterStore.find(parameterName) != mInternals->mParameterStore.end()) {
        result = mInternals->ROS2ParamMsgToParameter(mInternals->mParameterStore[parameterName]).get_type_name();
    } else {
        vtkWarningMacro(<< "Parameter " << parameterName << " is not tracked");
        result = "";  // does not exist
    }
    return result;
}

// A method that prints the value of a parameter. If the parameter is not set or if the parameter is not tracked, then it returns false.
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

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. 
If the Parameter is not tracked or if the parameter value is not set, it returns false with output value of result variable = false */
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

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. 
If the Parameter is not tracked or if the parameter value is not set, it returns false with output value of result variable = 0 */
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

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method.
If the Parameter is not tracked or if the parameter value is not set, it returns false with output value of result variable = 0 */
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

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method.
If the Parameter is not tracked or if the parameter value is not set, it returns false with output value of result variable = "" */
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

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. 
If the Parameter is not tracked or if the parameter value is not set, it returns false with output value of result variable = {} */
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

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. 
If the Parameter is not tracked or if the parameter value is not set, it returns false with output value of result variable = {} */
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

/*! Users should always make sure the parameterName exists and the parameter type is a string with GetParameterType before calling this method. 
If the Parameter is not tracked or if the parameter value is not set, it returns false with output value of result variable = {} */
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
    // add all parameter names from mParameterStore to mTrackedNodeNames
    for (auto it = mInternals->mParameterStore.begin(); it != mInternals->mParameterStore.end(); ++it) {
        mTrackedParameterNames.push_back(it->first);
    }
    Superclass::WriteXML(of, nIndent);  // This will take care of referenced nodes
    vtkMRMLWriteXMLBeginMacro(of);
    vtkMRMLWriteXMLStdStringMacro(MRMLNodeName, mMRMLNodeName);
    vtkMRMLWriteXMLStdStringMacro(TrackedNodeName, mTrackedNodeName);
    vtkMRMLWriteXMLStdStringVectorMacro(trackedParameterNames, mTrackedParameterNames)
    vtkMRMLWriteXMLEndMacro();
}

void vtkMRMLROS2ParameterNode::ReadXMLAttributes(const char **atts) {
    int wasModifying = this->StartModify();
    Superclass::ReadXMLAttributes(atts);  // This will take care of referenced nodes
    vtkMRMLReadXMLBeginMacro(atts);
    vtkMRMLReadXMLStdStringMacro(nodeName, mMRMLNodeName);
    vtkMRMLReadXMLStdStringMacro(trackedParameterNames, mTrackedParameterNames);
    vtkMRMLReadXMLEndMacro();
    this->EndModify(wasModifying);
    // add an empty parameter msg corresponding to each tracked node name to mParameterStore
    for (auto parameterName : mTrackedParameterNames) {
        mInternals->mParameterStore[parameterName] = ROS2ParamtoROS2ParamMsg(rclcpp::Parameter(parameterName));
    }
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
