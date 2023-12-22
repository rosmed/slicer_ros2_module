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
  return mInternals->lastValidResponseTable;
}

bool vtkMRMLROS2ServiceNode::GetLastResponse(vtkSmartPointer<vtkTable> &output) {
  if (mInternals->lastValidResponseTable) {
    output = mInternals->lastValidResponseTable;
    return true;
  }
  vtkErrorMacro("GetLastResponse: No valid response available");
  return false;
}

// Common functionality for client initialization and request check
bool vtkMRMLROS2ServiceNode::InitializeRequest() {
  if (mInternals->isRequestInProgress) {
    vtkErrorMacro(<< "ServiceNode::Request: request already in progress");
    return false;
  }
  std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> client = mInternals->mServiceClient;
  if (!client) {
    vtkErrorMacro(<< "ServiceNode::Request: service client is not initialized");
    return false;
  }
  return true;
}

// Asynchronous request sending
void vtkMRMLROS2ServiceNode::SendAsyncRequest() {
  if (!InitializeRequest()) return;
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  std::cerr << "ServiceNode::SendAsyncRequest sending request" << std::endl;
  mInternals->mServiceClient->async_send_request(request, std::bind(&vtkMRMLROS2ServiceInternals::service_callback, mInternals, std::placeholders::_1));
  mInternals->isRequestInProgress = true;
}

// Blocking request sending
void vtkMRMLROS2ServiceNode::SendBlockingRequest(unsigned int wait_time_ms) {
  if (!InitializeRequest()) return;
  vtkWarningMacro(<< "This is a blocking request. It will block the main thread (which Slicer uses) until the request is completes or times out in " << wait_time_ms << " ms.");

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  std::cerr << "ServiceNode::SendBlockingRequest sending request" << std::endl;

  std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>> future = mInternals->mServiceClient->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(wait_time_ms)) == std::future_status::ready){ //FIXME: there is a bug here
    std::shared_ptr<std_srvs::srv::Trigger::Response> response = future.get();
    mInternals->ProcessResponse(response);
  } else {
    vtkErrorMacro(<< "ServiceNode::SendBlockingRequest: request timed out");
  }
  mInternals->isRequestInProgress = false;
}

// void vtkMRMLROS2ServiceNode::SendRequest() {

//     if(mInternals->isRequestInProgress) {
//         vtkErrorMacro(<< "ServiceNode::SendRequest: request already in progress");
//         return;
//     }

//     std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> client = mInternals->mServiceClient;
//     if (!client) {
//         vtkErrorMacro(<< "ServiceNode::SendRequest: service client is not initialized");
//     }

//     mInternals->isRequestInProgress = true;
//     auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//     std::cerr << "ServiceNode::SendRequest sending request" << std::endl;
//     std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>> future = client->
//                                                                       async_send_request(request, std::bind(&vtkMRMLROS2ServiceInternals::service_callback, mInternals, std::placeholders::_1));
    
// }

// void vtkMRMLROS2ServiceNode::SendRequestBlocking(unsigned int wait_time_ms ) {
//     if(mInternals->isRequestInProgress) {
//         vtkErrorMacro(<< "ServiceNode::SendRequestBlocking: request already in progress");
//         return;
//     }

//     std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> client = mInternals->mServiceClient;
//     if (!client) {
//         vtkErrorMacro(<< "ServiceNode::SendRequestBlocking: service client is not initialized");
//         return;
//     }

//     auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//     vtkSmartPointer<vtkTable> responseTable = vtkTable::New();
//     std::cerr << "ServiceNode::SendRequestBlocking sending request" << std::endl;

//     // Send request synchronously and wait for the response
//     auto future = client->async_send_request(request);
//     if(future.wait_for(std::chrono::milliseconds(wait_time_ms)) == std::future_status::ready) {
//         std::shared_ptr<std_srvs::srv::Trigger::Response> response = future.get();

//         vtkROS2ToSlicer(*response, responseTable);

//         std::cerr << "ServiceNode::SendRequestBlocking: Value stored in the table + received response: " 
//                   << responseTable->GetValueByName(0, "message") << std::endl;

//         mInternals->lastValidResponse = response;
//         mInternals->lastValidResponseTable = responseTable;
//     } else {
//         vtkErrorMacro(<< "ServiceNode::SendRequestBlocking: request timed out");
//     }

//     mInternals->isRequestInProgress = false;
// }



// vtkTable* vtkMRMLROS2ServiceNode::SendRequestBlocking() {
//     auto client = mInternals->mServiceClient;
//     auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

//     std::cerr << "ServiceNode::SendRequest sending request" << std::endl;
    
//     vtkSmartPointer<vtkTable> result = vtkTable::New();
//         // Asynchronously send request
//     auto future = client->async_send_request(request);
    
    
//   if (rclcpp::spin_until_future_complete(mInternals->mROS2Node, future) != // TODO: keep as a feature
//     rclcpp::FutureReturnCode::SUCCESS)
//   {
//     std::cerr << "Failed call service toggle_state" << std::endl;
//   } else {
//     auto response = future.get();
//     std::cerr << "ServiceNode::SendRequest got response" << response->message << std::endl;

//     vtkSmartPointer<vtkStringArray> messageArray = vtkSmartPointer<vtkStringArray>::New();
//     messageArray->SetName("message");
//     result->AddColumn(messageArray);

//     vtkSmartPointer<vtkIntArray> successArray = vtkSmartPointer<vtkIntArray>::New();
//     successArray->SetName("success");
//     result->AddColumn(successArray);

//     result->SetNumberOfRows(1);
//     result->SetValue(0, 0, vtkVariant(response->message));
//     result->SetValue(0, 1, vtkVariant(response->success));

//   }

  

//   std::cerr << "ServiceNode::SendRequest got response" << std::endl;

//   return result;
    
// }


// // Setting up the service event subscriber. If the service is ready, add all monitored service values to the service server.
// bool vtkMRMLROS2ServiceNode::Spin()
// {
//   // if it is already initialized, return true (was completed in a previous spin)
//   if (this->mIsServiceServerReady) {
//     return true;
//   }

//   if (!mInternals->mServiceClient->service_is_ready()) {
//     // Print warning macros only once in every 10000 spins
//     if (mInternals->serviceNotReadyCounter++ % 10000 == 0) {
//       vtkWarningMacro(<< "Spin: service service for " << this->mMonitoredNodeName << " is not ready. Please verify if the node is running.");
//     }
//     return false;
//   }

//   this->mIsServiceServerReady = true;
//   // print that the service node is ready for current node
//   vtkDebugMacro(<< "Spin: service node for " << this->mMonitoredNodeName << " is ready");

//   // get a vector of std::string from the map
//   std::vector<std::string> serviceNames;
//   for (auto &service : mInternals->mServiceStore) {
//     serviceNames.push_back(service.first);
//   }

//   for (auto &service : mInternals->mServiceStore) {
//     // use get_services to get the service value from the service server
//     auto services_future =
//       mInternals->mServiceClient->get_services({service.first},
//                                                    std::bind(&vtkMRMLROS2ServiceInternals::GetServicesCallback,
//                                                              mInternals, std::placeholders::_1));
//   }

//   // Setting up the service event subscriber.
//   mInternals->mServiceEventSubscriber
//     = mInternals->mServiceClient
//     ->on_service_event(std::bind(&vtkMRMLROS2ServiceInternals::ServiceEventCallback, mInternals, std::placeholders::_1));

//   return true;
// }


// Check if service node has been added to Ros2Node
bool vtkMRMLROS2ServiceNode::IsAddedToROS2Node(void) const
{
  return mInternals->mServiceClient != nullptr;
}


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


// // Check if service server is ready
// bool vtkMRMLROS2ServiceNode::IsMonitoredNodeReady(void) const
// {
//   //todo: check if this is efficient - potentially add it to spin
//   return mInternals->mServiceClient->service_is_ready();
// }


// bool vtkMRMLROS2ServiceNode::AddService(const std::string &serviceName)
// {
//   if (mInternals->mServiceStore.find(serviceName) != mInternals->mServiceStore.end()) {
//     vtkWarningMacro(<< "AddService: service " << serviceName << " already exists");
//     return false;
//   }
//   mInternals->mServiceStore[serviceName] = rcl_interfaces::msg::Service();
//   if (!mInternals->mServiceClient->service_is_ready()) {
//     vtkWarningMacro(<< "AddService: service node for " << this->mMonitoredNodeName << " doesn't seem to be available");
//     // return false;
//   } else {
//     auto services_future =
//       mInternals->mServiceClient->get_services({serviceName},
//                                                    std::bind(&vtkMRMLROS2ServiceInternals::GetServicesCallback, mInternals, std::placeholders::_1));
//   }
//   return true;
// }


// bool vtkMRMLROS2ServiceNode::RemoveService(const std::string &serviceName)
// {
//   if (mInternals->mServiceStore.find(serviceName) != mInternals->mServiceStore.end()) {
//     mInternals->mServiceStore.erase(serviceName);
//     return true;
//   } else {
//     vtkWarningMacro(<< "RemoveService: service " << serviceName << " is not monitored");
//     return false;
//   }
// }


// bool vtkMRMLROS2ServiceNode::IsServiceSet(const std::string &serviceName, bool noWarning) const
// {
//   if (mInternals->mServiceStore.find(serviceName) != mInternals->mServiceStore.end()) {
//     return mInternals->ROS2ParamMsgToService(mInternals->mServiceStore[serviceName]).get_type() != rclcpp::ServiceType::PARAMETER_NOT_SET;
//   } else {
//     if (!noWarning) {
//       vtkWarningMacro(<< "RemoveService: service " << serviceName << " is not monitored");
//     }
//     return false;
//   }
// }


// bool vtkMRMLROS2ServiceNode::GetServiceType(const std::string &serviceName, std::string &result)
// {
//   result = "";
//   if (!CheckServiceExistsAndIsSet(serviceName)) {
//     return false;
//   }
//   result = mInternals->ROS2ParamMsgToService(mInternals->mServiceStore[serviceName]).get_type_name();
//   return true;
// }


// std::string vtkMRMLROS2ServiceNode::GetServiceType(const std::string &serviceName)
// {
//   std::string result = "";
//   GetServiceType(serviceName, result);
//   return result;
// }


// // A method that prints the value of a service. If the service is not set or if the service is not monitored, then it returns false.
// std::string vtkMRMLROS2ServiceNode::PrintService(const std::string &serviceName)
// {
//   std::string result = "service not found";
//   if (!CheckServiceExistsAndIsSet(serviceName)) {
//     return result;
//   }
//   try {
//     result = mInternals->ROS2ParamMsgToService(mInternals->mServiceStore[serviceName]).value_to_string();
//   } catch (const std::runtime_error &e) {
//     vtkErrorMacro(<< "PrintService: service " << serviceName << " value cannot be printed: " << e.what());
//   }
//   return result;
// }


// bool vtkMRMLROS2ServiceNode::GetServiceAsInteger(const std::string &serviceName, int &result)
// {
//   result = 0;
//   if (!CheckServiceExistsAndIsSet(serviceName)) {
//     return false;
//   }
//   try {
//     result = mInternals->ROS2ParamMsgToService(mInternals->mServiceStore[serviceName]).as_int();
//   } catch (const std::runtime_error &e) {
//     vtkErrorMacro(<< "GetServiceAsInteger: caught exception for service " << serviceName << ": " << e.what());
//     return false;
//   }
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
