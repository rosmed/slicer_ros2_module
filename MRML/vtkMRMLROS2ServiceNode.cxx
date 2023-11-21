#include <vtkMRMLROS2ServiceNode.h>

#include <vtkMRMLROS2Utils.h>
#include <vtkMRMLROS2ServiceInternals.h>

#include <rclcpp/rclcpp.hpp>
#include "turtlesim/srv/Spawn.hpp"
#include <stdexcept>
#include <utility>

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

  mInternals->mServiceClient = nodePointer->create_client<turtlesim::srv::Spawn>("/spawn");

  //TODO: maybe needs a spin?

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
  return true;
}

void vtkMRMLROS2ServiceNode::spawn_turtle(float x, float y, float theta, const std::string& name) {
    auto client = mInternals->mServiceClient
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = x;
    request->y = y;
    request->theta = theta;
    request->name = name;

    auto result = client->async_send_request(request);
    // Handle response in a callback or wait for the response
}




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
