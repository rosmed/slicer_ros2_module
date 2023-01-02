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
    mParameterSubscriber = std::make_shared<rclcpp::ParameterEventHandler>(nodePointer);

    auto cb = [this, nodePointer](const rcl_interfaces::msg::ParameterEvent &event)
    {
        // Obtain list of parameters that changed
        const std::string eventNodeName(event.node.c_str());
        if(this->mTrackedNodes.find(eventNodeName) != mTrackedNodes.end()){
          auto parameterList = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
          // Iterate through every parameter in the list
          for (const auto &p : parameterList)
          { 
              this->mAllParametersCount++;
              const std::string parameterName(p.get_name());
              const ParameterKey parameterPair = std::make_pair(eventNodeName, parameterName); // map of maps
              if(this->mParameterStore.find(parameterPair) != this->mParameterStore.end()){
                std::cerr << "Tracked Parameter (Updated)"  << parameterPair.first << " " << parameterPair.second << " " << p.value_to_string() << std::endl;
                this->mParameterStore[parameterPair] = p;
                this->mTrackedParametersCount++;
              }
          }
        }
    };

    cb_handle = mParameterSubscriber->add_parameter_event_callback(cb); // add the callback to the Subscriber

    rosNodePtr->SetNthNodeReferenceID("parameter",
				      rosNodePtr->GetNumberOfNodeReferences("parameter"),
				      mMRMLNode->GetID());
              
    mMRMLNode->SetNodeReferenceID("node", nodeId);

    return true;

  }

  bool IsAddedToROS2Node(void) const {
    return (mParameterSubscriber != nullptr);
  }

// modify to use pairs
  bool AddParameter(const std::string &nodeName, const std::string &parameterName, std::string & warningMessage) {
    ParameterKey parameterPair = std::make_pair(nodeName, parameterName);
    if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
      warningMessage = "Parameter already tracked";
    } else {
      if (mTrackedNodes.find(nodeName) == mTrackedNodes.end()) {
        mTrackedNodes.emplace(nodeName,1);
      } else {
        mTrackedNodes[nodeName]++;
      }
      mParameterStore.emplace(parameterPair, emptyParameter);
    }
    return true;
  }

  std::string GetParameterType(const std::string &nodeName, const std::string &parameterName, std::string & warningMessage){
  ParameterKey parameterPair = std::make_pair(nodeName, parameterName);
  if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
    return mParameterStore[parameterPair].get_type_name();
  } else {
    warningMessage =  "nodeName : " + nodeName + ":" + parameterName + "is not tracked"; 
    return ""; //does not exist
  }
}

bool PrintParameterValue(const ParameterKey & parameterPair, std::string & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
      result = mParameterStore[parameterPair].value_to_string();
      } catch (const std::runtime_error& e) {
        errorMessage = "PrintParameterValue caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    return false;
}

bool GetParameterAsString(const ParameterKey & parameterPair, std::string & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
      result = mParameterStore[parameterPair].as_string();
      } catch (const std::runtime_error & e) {
        errorMessage = "GetParameterAsString caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    // print error message
    return false;
}

bool GetParameterAsInteger(const ParameterKey & parameterPair, int & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
      result = mParameterStore[parameterPair].as_int();
      } catch (const std::runtime_error & e) {
        errorMessage = "GetParameterAsInteger caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    return false;
}

void listTrackedParameters(){ // rename GetParameterList -> vector<ParameterKeys>
    for (const auto& [key, value] : mParameterStore) {
        std::cerr << "-->" << key.first << ", " << key.second << " -- " << value.value_to_string() << std::endl; 
    }
}


//TODO: 

  // add documentation in header files - empty if "" else a param type (list them)
  // document the public and protected methods and data members
  // DeleteParameters()
  // Read and Write XML
  // print all tracked 

protected:
  vtkMRMLROS2ParameterNode * mMRMLNode;
  std::shared_ptr<rclcpp::ParameterEventHandler> mParameterSubscriber = nullptr;
  std::shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle;
  std::map<ParameterKey , rclcpp::Parameter> mParameterStore; // Parameters  mParameters  this->Parameter  VTK: GetValue()  qt: getValue()
  std::unordered_map<std::string, int> mTrackedNodes; // change to map of counts
  rclcpp::Parameter emptyParameter;
  int mTrackedParametersCount = 0;
  int mAllParametersCount = 0;
  // 
};


#endif
// __vtkMRMLROS2ParameterInternals_h

