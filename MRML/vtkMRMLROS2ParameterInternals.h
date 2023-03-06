#ifndef __vtkMRMLROS2ParameterInternals_h
#define __vtkMRMLROS2ParameterInternals_h

// ROS2 includes
#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2NodeInternals.h>
#include <vtkMRMLScene.h>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <utility>  // for std::pair
// Added for modified event
#include <vtkCommand.h>
#include <vtkMRMLROS2ParameterNode.h>

class vtkMRMLROS2ParameterInternals {
   public:
    vtkMRMLROS2ParameterInternals(vtkMRMLROS2ParameterNode *mrmlNode) : mMRMLNode(mrmlNode) {
    }

  friend class vtkMRMLROS2ParameterNode;

  enum Events // todo: check with Laura
    {
      ParameterModifiedEvent = vtkCommand::UserEvent + 54
    };


   protected:
// Converting a ROS2 parameter message to a ROS2 parameter.
    static rclcpp::Parameter ROS2ParamMsgToParameter(const rcl_interfaces::msg::Parameter &parameter) {
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

// Converting a ROS2 parameter to a ROS2 parameter message.
    static rcl_interfaces::msg::Parameter ROS2ParamToParameterMsg(const rclcpp::Parameter &parameter) {
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

// A callback function that is called when the parameter server responds to the request for parameters.
    void GetParametersCallback(std::shared_future<std::vector<rclcpp::Parameter>> future) {
        try {
            auto result = future.get();
            for (const auto &param : result) {
                mParameterStore[param.get_name()] = ROS2ParamToParameterMsg(param);
            }
            mMRMLNode->InvokeCustomModifiedEvent(ParameterModifiedEvent);
        } catch (std::exception &e) {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    }

// A callback function that is called when the parameter server responds to the request for parameters.
    void ParameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
        std::cerr << "New parameter event!" << std::endl;
        // Iterate over the new parameters
        for (const auto &new_param : event->new_parameters) {
            std::cerr << "New parameter: " << new_param.name.c_str() << std::endl;
            // rclcpp::Parameter param(new_param);
            mParameterStore[new_param.name] = new_param;
            mMRMLNode->InvokeCustomModifiedEvent(ParameterModifiedEvent);
        }
        // Iterate over the changed parameters
        for (const auto &changed_param : event->changed_parameters) {
            std::cerr << "Changed parameter: " << changed_param.name.c_str() << std::endl;
            // rclcpp::Parameter param(changed_param);
            mParameterStore[changed_param.name] = changed_param;
            mMRMLNode->InvokeCustomModifiedEvent(ParameterModifiedEvent);
        }
        // Iterate over the deleted parameters
        for (const auto &deleted_param : event->deleted_parameters) {
            std::cerr << "Deleted parameter: " << deleted_param.name.c_str() << std::endl;
            mParameterStore.erase(deleted_param.name);
        }
    }


   protected:
    
    //  A pointer to a ROS2 parameter event subscriber.
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr mParameterEventSubscriber = nullptr;
    vtkMRMLROS2ParameterNode *mMRMLNode;
    rclcpp::Parameter mEmptyParameter;
    // A map of parameters - specifically, parameter messages.
    std::map<std::string, rcl_interfaces::msg::Parameter> mParameterStore;
    std::shared_ptr<rclcpp::AsyncParametersClient> mParameterClient = nullptr;
    int serviceNotReadyCounter = 0;
};

#endif
// __vtkMRMLROS2ParameterInternals_h
