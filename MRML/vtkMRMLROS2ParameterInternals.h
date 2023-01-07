#ifndef __vtkMRMLROS2ParameterInternals_h
#define __vtkMRMLROS2ParameterInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <utility> // for std::pair
#include <vtkMRMLScene.h>
#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2NodeInternals.h>
#include <stdexcept>

class vtkMRMLROS2ParameterInternals
{
public:
  vtkMRMLROS2ParameterInternals(vtkMRMLROS2ParameterNode * mrmlNode):
    mMRMLNode(mrmlNode)
  {}

  typedef std::pair<std::string,std::string> ParameterKey; // pair: {nodeName, parameterName}

  bool AddToROS2Node(vtkMRMLScene * scene, const char * nodeId, const std::string &trackedNodeName, std::string & errorMessage) {

    vtkMRMLNode * rosNodeBasePtr = scene->GetNodeByID(nodeId);

    if (!rosNodeBasePtr) {
      errorMessage = "unable to locate node";
      return false;
    }

    vtkMRMLROS2NODENode * rosNodePtr = dynamic_cast<vtkMRMLROS2NODENode *>(rosNodeBasePtr);

    if (!rosNodePtr) {
      errorMessage = std::string(rosNodeBasePtr->GetName()) + " doesn't seem to be a vtkMRMLROS2NODENode";
      return false;
    }

    std::shared_ptr<rclcpp::Node> nodePointer = rosNodePtr->mInternals->mNodePointer;

    mParameterClient = std::make_shared<rclcpp::AsyncParametersClient>(nodePointer, trackedNodeName);

    std::chrono::seconds sec(1);

    while (!mParameterClient->wait_for_service(sec)) {
      if (!rclcpp::ok()) {
         std::cerr <<  "Interrupted while waiting for the service. Exiting." << std::endl;
      }
       std::cerr <<  "service not available, waiting again..." << std::endl;
    }

    mParameterEventSubscriber = mParameterClient->on_parameter_event(
          std::bind(&vtkMRMLROS2ParameterInternals::ParameterEventCallback, this, std::placeholders::_1));

    rosNodePtr->SetNthNodeReferenceID("parameter", rosNodePtr->GetNumberOfNodeReferences("parameter"),
				      mMRMLNode->GetID());
              
    mMRMLNode->SetNodeReferenceID("node", nodeId);

    return true;
  }

  bool IsAddedToROS2Node(void) const {
    return (mParameterEventSubscriber != nullptr);
  }

  bool AddParameter(const std::string &parameterName, std::string & warningMessage) {
    if(!this->mParameterClient->service_is_ready()) {
      warningMessage = "Async Parameters Client not ready";
      return false;
    }
    if (mParameterStore.find(parameterName) != mParameterStore.end()) {
      warningMessage = "Parameter already tracked";
      return false;
    } else {
      auto parameters_future = mParameterClient->get_parameters({parameterName}, 
          std::bind(&vtkMRMLROS2ParameterInternals::GetParameterCallback, this, std::placeholders::_1));
    }
    if(ROS2ParamMsgToParameter(mParameterStore[parameterName]).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      warningMessage = "Parameter not found";
      return false;
    }
    return true;
  }

  bool IsParameterSet(const std::string &parameterName, std::string & warningMessage) {
    if (mParameterStore.find(parameterName) != mParameterStore.end()) {
      return ROS2ParamMsgToParameter(mParameterStore[parameterName]).get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
    } else {
      warningMessage = "Parameter not tracked";
      return false;
    }
  }

  bool RemoveParameter(const std::string &parameterName, std::string & warningMessage) {
    if (mParameterStore.find(parameterName) != mParameterStore.end()) {
      mParameterStore.erase(parameterName);
      return true;
    } else {
      warningMessage = "Parameter not tracked";
      return false;
    }
  }

  std::string GetParameterType(const std::string &parameterName, std::string &result, std::string & warningMessage){
  if (mParameterStore.find(parameterName) != mParameterStore.end()) {
    result = ROS2ParamMsgToParameter(mParameterStore[parameterName]).get_type_name();
  } else {
    warningMessage =  "parameter : " + parameterName + " is not tracked"; 
    result = ""; //does not exist
  }
  return result;
}

bool PrintParameterValue(const std::string & parameterName, std::string & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterName) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;

      if ( ROS2ParamMsgToParameter(mParameterStore[parameterName]).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        errorMessage = "Parameter value not set";
        return false;
      }

      try {
        result = ROS2ParamMsgToParameter(mParameterStore[parameterName]).value_to_string();
      } catch (const std::runtime_error& e) {
        errorMessage = "PrintParameterValue caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }

      return parameterRetrievalStatus;
    }
    errorMessage = "Parameter not tracked";
    return false;
}

bool GetParameterAsBool(const std::string & parameterName, bool & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterName) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
        result = ROS2ParamMsgToParameter(mParameterStore[parameterName]).as_bool(); // if not set add another excep
      } catch (const std::runtime_error & e) {
        errorMessage = "GetParameterAsBoolean caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    errorMessage = "Parameter not tracked";
    return false;
}

bool GetParameterAsInteger(const std::string & parameterName, int & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterName) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
        result = ROS2ParamMsgToParameter(mParameterStore[parameterName]).as_int();
      } catch (const std::runtime_error & e) {
        errorMessage = "GetParameterAsInteger caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    errorMessage = "Parameter not tracked";
    return false;
}

bool GetParameterAsDouble(const std::string & parameterName, double & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterName) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
        result = ROS2ParamMsgToParameter(mParameterStore[parameterName]).as_double();
      } catch (const std::runtime_error & e) {
        errorMessage = "GetParameterAsDouble caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    errorMessage = "Parameter not tracked";
    return false;
}

bool GetParameterAsString(const std::string & parameterName, std::string & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterName) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
        result = ROS2ParamMsgToParameter(mParameterStore[parameterName]).as_string();
      } catch (const std::runtime_error & e) {
        errorMessage = "GetParameterAsString caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    errorMessage = "Parameter not tracked";
    return false;
}

// bool GetParameterAsVectorOfBools(const std::string & parameterName, std::vector<bool> & result, std::string & errorMessage) {
//     if (mParameterStore.find(parameterName) != mParameterStore.end()) {
//       bool parameterRetrievalStatus = true;
//       try {
//         result = ROS2ParamMsgToParameter(mParameterStore[parameterName]).as_bool_array();
//       } catch (const std::runtime_error & e) {
//         errorMessage = "GetParameterAsVectorOfBool caught exception :";
//         errorMessage.append(e.what());
//         parameterRetrievalStatus = false;
//       }
//       return parameterRetrievalStatus;
//     }
//     errorMessage = "Parameter not tracked";
//     return false;
// }

bool GetParameterAsVectorOfIntegers(const std::string & parameterName, std::vector<int64_t> & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterName) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
        result = ROS2ParamMsgToParameter(mParameterStore[parameterName]).as_integer_array();
      } catch (const std::runtime_error & e) {
        errorMessage = "GetParameterAsVectorOfInteger caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    errorMessage = "Parameter not tracked";
    return false;
}

bool GetParameterAsVectorOfDoubles(const std::string & parameterName, std::vector<double> & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterName) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
        result = ROS2ParamMsgToParameter(mParameterStore[parameterName]).as_double_array();
      } catch (const std::runtime_error & e) {
        errorMessage = "GetParameterAsVectorOfDouble caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    errorMessage = "Parameter not tracked";
    return false;
}

bool GetParameterAsVectorOfStrings(const std::string & parameterName, std::vector<std::string> & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterName) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
        result = ROS2ParamMsgToParameter(mParameterStore[parameterName]).as_string_array();
      } catch (const std::runtime_error & e) {
        errorMessage = "GetParameterAsVectorOfString caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    errorMessage = "Parameter not tracked";
    return false;
}

void listTrackedParameters(){ 
    for (const auto& [key, value] : mParameterStore) {

      auto param = ROS2ParamMsgToParameter(value); 

      std::cerr << "-->" << key << ", " << param.value_to_string() << std::endl; 
    }
}

std::vector<std::string> GetTrackedParametersList() {
  std::vector<std::string> ParametersList;
  for (const auto& [key, value] : mParameterStore) {
      ParametersList.push_back(key);
  }
  return ParametersList;
}

private:

  void GetParameterCallback(std::shared_future<std::vector<rclcpp::Parameter>> future)
  {
    try{
      auto result = future.get();
      auto param = result.at(0);
      mParameterStore[param.get_name()] = ROS2ParamToParameterMsg(param);
    } catch (std::exception &e) {
      std::cerr << "Exception: " << e.what() << std::endl;
    }

  }

  void ParameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {

    std::cerr << "new parameter event" << std::endl;
    // Iterate over the new parameters
    for (const auto & new_param : event->new_parameters) {
      std::cerr <<"New parameter: " << new_param.name.c_str()<< std::endl;
      // rclcpp::Parameter param(new_param);
      this->mParameterStore[new_param.name] = new_param;
    }
    // Iterate over the changed parameters
    for (const auto & changed_param : event->changed_parameters) {
      std::cerr <<"Changed parameter: " << changed_param.name.c_str()<< std::endl;
      // rclcpp::Parameter param(changed_param);
      mParameterStore[changed_param.name] = changed_param;
    }
    // Iterate over the deleted parameters
    for (const auto & deleted_param : event->deleted_parameters) {
      std::cerr <<"Deleted parameter: " << deleted_param.name.c_str()<< std::endl;
      mParameterStore.erase(deleted_param.name);
    }
  }

rclcpp::Parameter ROS2ParamMsgToParameter(const rcl_interfaces::msg::Parameter & parameter)
{
  rclcpp::Parameter param;
  switch (parameter.value.type) {
    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
      param = rclcpp::Parameter(parameter.name, parameter.value.bool_value);
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
      param = rclcpp::Parameter(parameter.name, parameter.value.integer_value);
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
      param = rclcpp::Parameter(parameter.name, parameter.value.double_value);
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
      param = rclcpp::Parameter(parameter.name, parameter.value.string_value);
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
      param = rclcpp::Parameter(parameter.name, parameter.value.byte_array_value);
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
      param = rclcpp::Parameter(parameter.name, parameter.value.bool_array_value);
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
      param = rclcpp::Parameter(parameter.name, parameter.value.integer_array_value);
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
      param = rclcpp::Parameter(parameter.name, parameter.value.double_array_value);
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
      param = rclcpp::Parameter(parameter.name, parameter.value.string_array_value);
      break;
    default:
      param = rclcpp::Parameter(parameter.name);
      break;
  }
  return param;
}

rcl_interfaces::msg::Parameter ROS2ParamToParameterMsg(const rclcpp::Parameter & parameter)
{
  rcl_interfaces::msg::Parameter param;
  param.name = parameter.get_name();
  switch (parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      param.value.bool_value = parameter.as_bool();
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      param.value.integer_value = parameter.as_int();
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      param.value.double_value = parameter.as_double();
      break;
    case rclcpp::ParameterType::PARAMETER_STRING:
      param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      param.value.string_value = parameter.as_string();
      break;
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY;
      param.value.byte_array_value = parameter.as_byte_array();
      break;
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
      param.value.bool_array_value = parameter.as_bool_array();
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
      param.value.integer_array_value = parameter.as_integer_array();
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
      param.value.double_array_value = parameter.as_double_array();
      break;
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
      param.value.string_array_value = parameter.as_string_array();
      break;
    default:
      param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
      break;
  }
  return param;
}

protected:
  vtkMRMLROS2ParameterNode * mMRMLNode;

  std::map<std::string , rcl_interfaces::msg::Parameter> mParameterStore; 
  rclcpp::Parameter mEmptyParameter;

  std::shared_ptr<rclcpp::AsyncParametersClient> mParameterClient;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr mParameterEventSubscriber = nullptr;
  // 

};


#endif
// __vtkMRMLROS2ParameterInternals_h

