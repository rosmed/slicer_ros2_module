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
#include "std_srvs/srv/set_bool.hpp"
#include "turtlesim/srv/spawn.hpp"

class vtkMRMLROS2ServiceInternals {
public:
  vtkMRMLROS2ServiceInternals(vtkMRMLROS2ServiceNode *mrmlNode) : mMRMLNode(mrmlNode) {
  }

  friend class vtkMRMLROS2ServiceNode;

  enum Events // todo: check with Laura
    {
     ServiceModifiedEvent = vtkCommand::UserEvent + 54
    };


// protected:
//   // Converting a ROS2 service message to a ROS2 service.
//   static rclcpp::Service ROS2ParamMsgToService(const rcl_interfaces::msg::Service &service) {
//     rclcpp::Service param;
//     return param;
//   }

//   // Converting a ROS2 service to a ROS2 service message.
//   static rcl_interfaces::msg::Service ROS2ParamToServiceMsg(const rclcpp::Service &service) {
//     rcl_interfaces::msg::Service param;
//     return param;
//   }

//   // A callback function that is called when the service server responds to the request for services.
//   void GetServicesCallback(std::shared_future<std::vector<rclcpp::Service>> future) {
//     try {
//       auto result = future.get();
//       for (const auto &param : result) {
//         mServiceStore[param.get_name()] = ROS2ParamToServiceMsg(param);
//       }
//       mMRMLNode->InvokeCustomModifiedEvent(ServiceModifiedEvent);
//     } catch (std::exception &e) {
//       std::cerr << "Exception: " << e.what() << std::endl;
//     }
//   }

//   // A callback function that is called when the service server responds to the request for services.
//   void ServiceEventCallback(const rcl_interfaces::msg::ServiceEvent::SharedPtr event) {
//     // Iterate over the new services
//     for (const auto &new_param : event->new_services) {
//       // rclcpp::Service param(new_param);
//       mServiceStore[new_param.name] = new_param;
//       mMRMLNode->InvokeCustomModifiedEvent(ServiceModifiedEvent);
//     }
//     // Iterate over the changed services
//     for (const auto &changed_param : event->changed_services) {
//       // rclcpp::Service param(changed_param);
//       mServiceStore[changed_param.name] = changed_param;
//       mMRMLNode->InvokeCustomModifiedEvent(ServiceModifiedEvent);
//     }
//     // Iterate over the deleted services
//     for (const auto &deleted_param : event->deleted_services) {
//       mServiceStore.erase(deleted_param.name);
//     }
//   }


protected:

  std::shared_ptr<rclcpp::Client<turtlesim::srv::Spawn>> mServiceClient = nullptr;
  //  A pointer to a ROS2 service event subscriber.
  // rclcpp::Subscription<rcl_interfaces::msg::ServiceEvent>::SharedPtr mServiceEventSubscriber = nullptr;
  vtkMRMLROS2ServiceNode *mMRMLNode;
  // rclcpp::Service mEmptyService;
  // A map of services - specifically, service messages.
  // std::map<std::string, rcl_interfaces::msg::Service> mServiceStore;
  // std::shared_ptr<rclcpp::AsyncServicesClient> mServiceClient = nullptr;
  int serviceNotReadyCounter = 0;
};

#endif

// __vtkMRMLROS2ServiceInternals_h
