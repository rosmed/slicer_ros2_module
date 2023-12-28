#ifndef __vtkMRMLROS2ServiceInternals_h
#define __vtkMRMLROS2ServiceInternals_h

// ROS2 includes
#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2NodeInternals.h>
#include <vtkMRMLScene.h>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <utility>  // for std::pair
// Added for modified event
#include <vtkCommand.h>
#include <vtkMRMLROS2ServiceNode.h>
#include "std_srvs/srv/trigger.hpp"
#include <vtkROS2ToSlicer.h>

class vtkMRMLROS2ServiceInternals {
public:
  vtkMRMLROS2ServiceInternals(vtkMRMLROS2ServiceNode *mrmlNode) : mMRMLNode(mrmlNode) {
  }

  friend class vtkMRMLROS2ServiceNode;

  enum Events // todo: check with Laura
    {
     ServiceModifiedEvent = vtkCommand::UserEvent + 54
    };

  // enum class ServiceResponseStatus 
  // {
  //     Success,
  //     Timeout,
  //     Error
  // };


protected:

  void service_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {

    this->isRequestInProgress = false;
    std::shared_ptr<std_srvs::srv::Trigger::Response> service_response_ = future.get();    

    try {
      this->ProcessResponse(service_response_);
    } catch (const std::exception& e) {
      this->ProcessErrorResponse(e.what());
    }
  }

  // Process response
  void ProcessResponse(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    vtkSmartPointer<vtkTable> responseTable = vtkTable::New();
    vtkROS2ToSlicer(*response, responseTable);
    std::cerr << "ServiceNode::ProcessResponse: Value stored in the table + received response: " 
              << responseTable->GetValueByName(0, "message") << std::endl;

    this->lastResponse = response;
    this->lastResponseTable = responseTable;
    this->lastResponseSuccess = true;
  }

  void ProcessErrorResponse(const std::string& error_message) {
    std::cerr << "Received error response: " << error_message << std::endl;

    this->lastResponse = nullptr;
    this->lastResponseTable = vtkTable::New();
    this->lastResponseSuccess = false;
  }



protected:

  std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> mServiceClient = nullptr;

  std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>> mServiceResponseFuture;
  bool isRequestInProgress = false;

  std::shared_ptr<std_srvs::srv::Trigger::Response> lastResponse = nullptr;
  vtkSmartPointer<vtkTable> lastResponseTable = vtkTable::New();
  bool lastResponseSuccess = false;


  // A pointer to a ROS2 node.
  std::shared_ptr<rclcpp::Node> mROS2Node = nullptr;

  vtkMRMLROS2ServiceNode *mMRMLNode;

  int serviceNotReadyCounter = 0;
};

#endif

// __vtkMRMLROS2ServiceInternals_h
