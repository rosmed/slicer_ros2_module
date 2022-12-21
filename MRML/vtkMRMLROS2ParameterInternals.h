#ifndef __vtkMRMLROS2ParameterInternals_h
#define __vtkMRMLROS2ParameterInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <utility> // for std::pair
#include <vtkMRMLScene.h>
#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2NodeInternals.h>

class vtkMRMLROS2ParameterInternals
{
public:
  vtkMRMLROS2ParameterInternals(vtkMRMLROS2ParameterNode * mrmlNode):
    mMRMLNode(mrmlNode)
  {}

  typedef std::pair<std::string,std::string> ParameterKey; // pair: {nodeName, parameterName}

  bool AddToROS2Node(vtkMRMLScene * scene, const char * nodeId, std::string & errorMessage) {

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
    parameterSubscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(nodePointer);

    auto cb = [this, nodePointer](const rcl_interfaces::msg::ParameterEvent &event)
    {
        // Obtain list of parameters that changed
          std::string eventNodeName(event.node.c_str());

        if(this->trackedNodes.find(eventNodeName) != trackedNodes.end()){
          auto parameterList = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
            // Iterate through every parameter in the list
          for (auto &p : parameterList)
          { 
              this->allParametersCount++;
              std::string parameterName(p.get_name());
              ParameterKey parameterPair = std::make_pair(eventNodeName, parameterName);
              std::cerr << " " << std::endl;
              // std::cerr << "Received parameter update "  << parameterPair.first << " " << parameterPair.second;
              if(this->parameterStore.find(parameterPair) != this->parameterStore.end()){
                std::cerr << std::endl;
                std::cerr << "Tracked Parameter (Updated)"  << parameterPair.first << " " << parameterPair.second << " " << p.value_to_string() << std::endl;
                this->parameterStore[parameterPair] = p;
                this->trackedParametersCount++;
              } else {
                // std::cerr << " : Untracked parameter (Ignored)"  << parameterPair.first << " " << parameterPair.second << std::endl;
              }
          }
        }
    };

    cb_handle = parameterSubscriber_->add_parameter_event_callback(cb);

    rosNodePtr->SetNthNodeReferenceID("parameter",
				      rosNodePtr->GetNumberOfNodeReferences("parameter"),
				      mMRMLNode->GetID());
              
    mMRMLNode->SetNodeReferenceID("node", nodeId);

    return true;

  }

  bool IsAddedToROS2Node(void) const {
    return (parameterSubscriber_ != nullptr);
  }

  std::string GetParameterType(std::string nodeName, std::string parameterName){
    ParameterKey parameterPair = std::make_pair(nodeName, parameterName);
    if (parameterStore.find(parameterPair) != parameterStore.end()) {
      std::cerr << "stored val =" << parameterStore[parameterPair].get_type_name() <<std::endl; 
      return parameterStore[parameterPair].value_to_string();
    } else {
      std::cerr << "nodeName : " << nodeName << ":" << parameterName << "is not tracked " << std::endl;
      return "undefined";
    }
  }

  bool AddParameter(std::string nodeName, std::string parameterName){
    ParameterKey parameterPair = std::make_pair(nodeName, parameterName);
    if (parameterStore.find(parameterPair) != parameterStore.end()) {
      std::cerr << "Parameter already tracked" <<std::endl;
      return true;
    } else {
      trackedNodes.insert(nodeName);
      parameterStore.emplace(parameterPair, emptyParameter);
      return true;
    }
  }

  void listTrackedParameters(){
      for (const auto& [key, value] : parameterStore) {
          std::cerr << "-->" << key.first << ", " << key.second << " -- " << value.value_to_string() << std::endl; 
      }
  }

  // std::string GetParameterString(std::string nodeName, std::string parameterName){

  // }

  // std::int GetParameterInt(std::string nodeName, std::string parameterName){

  // }

protected:
  vtkMRMLROS2ParameterNode * mMRMLNode;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameterSubscriber_ = nullptr;
  std::shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle;
  std::map<ParameterKey , rclcpp::Parameter> parameterStore; 
  rclcpp::Parameter emptyParameter;
  std::unordered_set<std::string> trackedNodes;
  int trackedParametersCount = 0;
  int allParametersCount = 0;
};


#endif
// __vtkMRMLROS2ParameterInternals_h

