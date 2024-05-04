#include <vtkMRMLROS2ServiceNode.h>

#include <vtkMRMLROS2Utils.h>
#include <vtkMRMLROS2ServiceInternals.h>

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <stdexcept>
#include <utility>
#include <chrono>

#include <vtkTable.h>
#include <vtkSmartPointer.h>
#include <vtkVariant.h>
#include <vtkStringArray.h>
#include <vtkIntArray.h>
#include <vtkROS2ToSlicer.h>
// #include "vtkBoolString.h"

vtkStandardNewMacro(vtkMRMLROS2ServiceNode);

vtkMRMLROS2ServiceNode::vtkMRMLROS2ServiceNode()
{
  mInternals = std::make_shared<vtkMRMLROS2ServiceInternals>(this);
}


vtkMRMLROS2ServiceNode::~vtkMRMLROS2ServiceNode()
{
}


void vtkMRMLROS2ServiceNode::PrintSelf(ostream &os, vtkIndent indent)
{
  Superclass::PrintSelf(os, indent);
  // Custom prints
  os << indent << "Monitored Node Name: " << mMonitoredNodeName << "\n";
  os << indent << "MRML Node Name: " << mMRMLNodeName << "\n";
  // os << indent << "ROS type: " << mInternals->GetROSType() << "\n";

  // print contents of mServiceStore
  // os << indent << "Monitored Services : " << "\n";
  // for (const auto &[key, value] : mInternals->mServiceStore) {
  //   auto param = mInternals->ROS2ParamMsgToService(value);
  //   os << indent << indent << key << ": " << param.value_to_string() << "\n";
  // }
}


vtkMRMLNode *vtkMRMLROS2ServiceNode::CreateNodeInstance(void)
{
  return SelfType::New();
}


const char *vtkMRMLROS2ServiceNode::GetNodeTagName(void)
{
  return "ROS2Service";
}


bool vtkMRMLROS2ServiceNode::AddToROS2Node(const char * nodeId, const std::string & monitoredNodeName)
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

  mInternals->mServiceClient = nodePointer->create_client<std_srvs::srv::Trigger>("toggle_state");

  // // create a service client
  // mInternals->mServiceClient = std::make_shared<rclcpp::AsyncServicesClient>(nodePointer, monitoredNodeName);
  // // add this service node to the ROS node, so that it can be spin in the same thread as the ROS node
  // mrmlROSNodePtr->mServiceNodes.push_back(this);
  // mrmlROSNodePtr->SetNthNodeReferenceID("service",
  //                                       mrmlROSNodePtr->GetNumberOfNodeReferences("service"),
  //                                       this->GetID());
  this->SetNodeReferenceID("node", nodeId);
  mrmlROSNodePtr->WarnIfNotSpinning("adding service client for \"" + monitoredNodeName + "\"");
  mInternals->mMRMLNode = this;
  mInternals->mROS2Node = nodePointer;
  return true;
}

vtkTable* vtkMRMLROS2ServiceNode::GetLastResponseAsTable() {
  if (!GetLastResponseStatus()) {
    vtkErrorMacro("GetLastResponseAsTable: No valid response available");
    return nullptr;
  }
  return mInternals->lastResponseTable;
}

bool vtkMRMLROS2ServiceNode::GetLastResponse(vtkSmartPointer<vtkTable> &output) {
  if (!GetLastResponseStatus()) {
    output = mInternals->lastResponseTable;
    return true;
  }
  vtkErrorMacro("GetLastResponse: No valid response available");
  return false;
}

bool vtkMRMLROS2ServiceNode::GetLastResponseStatus() {
  return mInternals->lastResponseSuccess;
}

// TODO : Make a IsRequestInProgress void const
// TODO : IsAddedtoROS also needs to be there. To be used in PreCheck

// Common functionality for client initialization and request check
bool vtkMRMLROS2ServiceNode::PreRequestCheck() { 
  // Get the shared pointer to the service client
  std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> client = mInternals->mServiceClient;
  // Check if the service client is initialized
  if (!client) {
    vtkErrorMacro(<< "ServiceNode::PreRequestCheck: service client is not initialized");
    return false;
  }
  // Check the state of the client to ensure the server is running
  if (!client->service_is_ready()) {
    vtkErrorMacro(<< "ServiceNode::PreRequestCheck: service server is not running");
    // server is not running
    mInternals->isRequestInProgress = false; // TODO: This is a hack to handle cases where the server fails midway and the Request is still in progress
    mInternals->ProcessErrorResponse("ServiceNode::PreRequestCheck: server is not running");
    return false;
  }
  // Check if a request is already in progress
  if (mInternals->isRequestInProgress) {
    vtkErrorMacro(<< "ServiceNode::PreRequestCheck: request already in progress");
    return false;
  }
  return true;
}

// Asynchronous request sending
void vtkMRMLROS2ServiceNode::SendAsyncRequest() {
  if (!PreRequestCheck()) return;
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  std::cerr << "ServiceNode::SendAsyncRequest sending request" << std::endl;
  mInternals->mServiceResponseFuture = mInternals->mServiceClient->async_send_request(request, std::bind(&vtkMRMLROS2ServiceInternals::service_callback, mInternals, std::placeholders::_1)).future;
  // TODO: Edge case where server dies before callback is called
  mInternals->isRequestInProgress = true;
}

// templating url : https://github.com/jhu-cisst/cisst_ros2_bridge/blob/main/include/cisst_ros2_bridge/mtsROSBridge.h

void vtkMRMLROS2ServiceNode::SendBlockingRequest(unsigned int wait_time_ms) {
  // Pre-request checks
  if (!PreRequestCheck()) return;
  vtkWarningMacro(<< "This is a blocking request. It will block the main thread (which Slicer uses) until the request completes or times out in " << wait_time_ms << " ms.");

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  std::cerr << "ServiceNode::SendBlockingRequest sending request" << std::endl;
  mInternals->isRequestInProgress = true;
  std::string error_string = "";
  try {
    // Sending request and waiting for response
    // std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>> future = mInternals->mServiceClient->async_send_request(request);
    std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>> future = mInternals->mServiceClient->async_send_request(request).future.share();
    mInternals->mServiceResponseFuture = future;
    auto resultState = rclcpp::spin_until_future_complete(mInternals->mROS2Node, future, std::chrono::milliseconds(wait_time_ms));
    
    switch (resultState) {
      case rclcpp::FutureReturnCode::SUCCESS:
        mInternals->ProcessResponse(future.get());
        break;
      case rclcpp::FutureReturnCode::TIMEOUT:
        error_string = "ServiceNode::SendBlockingRequest: request timed out";
        vtkErrorMacro(<< error_string);
        mInternals->ProcessErrorResponse(error_string);
        break;
      default:
        error_string = "ServiceNode::SendBlockingRequest: request failed";
        vtkErrorMacro(<< error_string);
        mInternals->ProcessErrorResponse(error_string);
        break;
    }
  } catch (const std::exception& e) {
    error_string = "ServiceNode::SendBlockingRequest: Exception occurred: " + std::string(e.what());
    vtkErrorMacro(<< error_string);
    mInternals->ProcessErrorResponse(error_string);
  }
  mInternals->isRequestInProgress = false;
}

 
// Check if service node has been added to Ros2Node
bool vtkMRMLROS2ServiceNode::IsAddedToROS2Node(void) const
{
  return mInternals->mServiceClient != nullptr;
}


// bool vtkMRMLROS2ServiceNode::testScalarHolder(void)
// {
//   vtkNew<vtkBoolString> scalarHolder;

//   // Set a scalar value
//   scalarHolder->SetValue(vtkVariant(42)); // Example with an integer
//   scalarHolder->SetValue(vtkVariant(3.14)); // Example with a double

//   // Retrieve and use the scalar value
//   vtkVariant value = scalarHolder->GetValue();
//   if (value.IsDouble()) {
//     double dblValue = value.ToDouble();
//     // Use dblValue as needed
//   } else if (value.IsInt()) {
//     int intValue = value.ToInt();
//     // Use intValue as needed
//   }
// }

// bool vtkMRMLROS2ServiceNode::RemoveFromROS2Node(const char *nodeId)
// {
//   std::string errorMessage;
//   vtkMRMLROS2NodeNode * rosNodePtr = vtkMRMLROS2::CheckROS2NodeExists(this, nodeId, errorMessage);
//   if (!rosNodePtr) {
//     vtkErrorMacro(<< "RemoveFromROS2Node: " << errorMessage);
//     return false;
//   }

//   if (!this->IsAddedToROS2Node()) {
//     vtkErrorMacro(<< "RemoveFromROS2Node: service node is not added to ROS2 node");
//     return false;
//   }

//   // remove the service node from the ROS node
//   auto it = std::find(rosNodePtr->mServiceNodes.begin(), rosNodePtr->mServiceNodes.end(), this);
//   if (it != rosNodePtr->mServiceNodes.end()) {
//     rosNodePtr->mServiceNodes.erase(it);
//   }

//   mInternals->mMRMLNode = nullptr;
//   this->SetNodeReferenceID("node", nullptr);
//   rosNodePtr->RemoveNthNodeReferenceID("service", rosNodePtr->GetNumberOfNodeReferences("service"));

//   mInternals->mServiceClient.reset();
//   return true;
// }





// void vtkMRMLROS2ServiceNode::WriteXML(std::ostream &of, int nIndent)
// {
//   // add all service names from mServiceStore to mMonitoredNodeNames
//   for (auto it = mInternals->mServiceStore.begin(); it != mInternals->mServiceStore.end(); ++it) {
//     MonitoredServiceNamesCache.push_back(it->first);
//   }
//   Superclass::WriteXML(of, nIndent);  // This will take care of referenced nodes
//   vtkMRMLWriteXMLBeginMacro(of);
//   vtkMRMLWriteXMLStdStringMacro(MRMLNodeName, mMRMLNodeName);
//   vtkMRMLWriteXMLStdStringMacro(MonitoredNodeName, mMonitoredNodeName);
//   vtkMRMLWriteXMLStdStringVectorMacro(monitoredServiceNames, MonitoredServiceNamesCache, std::vector);
//   vtkMRMLWriteXMLEndMacro();
//   // clear mMonitoredNodeNames
//   MonitoredServiceNamesCache.clear();
// }


// void vtkMRMLROS2ServiceNode::ReadXMLAttributes(const char **atts)
// {
//   int wasModifying = this->StartModify();
//   Superclass::ReadXMLAttributes(atts);  // This will take care of referenced nodes
//   vtkMRMLReadXMLBeginMacro(atts);
//   vtkMRMLReadXMLStdStringMacro(nodeName, mMRMLNodeName);
//   vtkMRMLReadXMLStdStringMacro(MonitoredNodeName, mMonitoredNodeName);
//   vtkMRMLReadXMLStdStringVectorMacro(monitoredServiceNames, MonitoredServiceNamesCache, std::vector);
//   vtkMRMLReadXMLEndMacro();
//   this->EndModify(wasModifying);
//   // add an empty service msg corresponding to each monitored node name to mServiceStore
//   for (const auto & serviceName : MonitoredServiceNamesCache) {
//     mInternals->mServiceStore[serviceName] = mInternals->ROS2ParamToServiceMsg(rclcpp::Service(serviceName));
//   }
// }


// void vtkMRMLROS2ServiceNode::UpdateScene(vtkMRMLScene *scene)
// {
//   Superclass::UpdateScene(scene);
//   int nbNodeRefs = this->GetNumberOfNodeReferences("node");
//   if (nbNodeRefs == 0) {
//     // assigned to the default ROS node
//     auto defaultNode = scene->GetFirstNodeByName("ros2:node:slicer");
//     if(!defaultNode){
//       vtkErrorMacro(<< "UpdateScene: default ros2 node unavailable. Unable to set reference for service \"" << GetName() << "\"");
//       return;
//     }
//     this->AddToROS2Node(defaultNode->GetID(), mMonitoredNodeName);
//   } else if (nbNodeRefs == 1) {
//     this->AddToROS2Node(this->GetNthNodeReference("node", 0)->GetID(), mMonitoredNodeName);
//   } else {
//     vtkErrorMacro(<< "UpdateScene: more than one ROS2 node reference defined for service \"" << GetName() << "\"");
//   }
// }


/* Custom Setter for the vector ServiceNamesList */
// void vtkMRMLROS2ServiceNode::SetMonitoredServiceNamesCache(const std::vector<std::string> &monitoredServiceNames)
// {
//   this->MonitoredServiceNamesCache.clear();
//   // iterate through the vector and add each service name to the list
//   for (const auto service : monitoredServiceNames) {
//     this->MonitoredServiceNamesCache.push_back(service);
//   }
//   this->Modified();
//   // this->InvokeCustomModifiedEvent(vtkMRMLROS2ServiceNode::InputDataModifiedEvent);
// }


// /* Custom Getter for the vector ServiceNamesList */
// std::vector<std::string> vtkMRMLROS2ServiceNode::GetMonitoredServiceNamesCache()
// {
//   std::vector<std::string> monitoredServiceNames;
//   for (const auto service : this->MonitoredServiceNamesCache) {
//     monitoredServiceNames.push_back(service);
//   }
//   return monitoredServiceNames;
// }


// bool vtkMRMLROS2ServiceNode::CheckServiceExistsAndIsSet(const std::string &serviceName) const
// {
//   if (mInternals->mServiceStore.find(serviceName) == mInternals->mServiceStore.end()) {
//     vtkErrorMacro(<< "CheckServiceExistsAndIsSet: service " << serviceName << " does not exist");
//     return false;
//   }
//   // if monitored but not set
//   if (mInternals->ROS2ParamMsgToService(mInternals->mServiceStore[serviceName]).get_type() == rclcpp::ServiceType::PARAMETER_NOT_SET) {
//     vtkErrorMacro(<< "CheckServiceExistsAndIsSet: service " << serviceName << " value is not set");
//     return false;
//   }
//   return true;
// }

// vtkStandardNewMacro(vtkBoolString);

// vtkBoolString::vtkBoolString() = default;
// vtkBoolString::~vtkBoolString() = default;
