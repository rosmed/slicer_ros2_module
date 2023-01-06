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

  bool AddParameter(const std::string &nodeName, const std::string &parameterName, std::string & warningMessage) {
    ParameterKey parameterPair = std::make_pair(nodeName, parameterName);
    if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
      warningMessage = "Parameter already tracked";
      return false;
    } else {
      if (mTrackedNodes.find(nodeName) == mTrackedNodes.end()) {
        mTrackedNodes.emplace(nodeName,1);
      } else {
        mTrackedNodes[nodeName]++;
      }
      mParameterStore.emplace(parameterPair, mEmptyParameter); //rclcpp::Parameter() - remove empty
      
    }
    return true;
  }

  bool RemoveParameter(const std::string &nodeName, const std::string &parameterName, std::string & warningMessage) {
    ParameterKey parameterPair = std::make_pair(nodeName, parameterName);
    if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
      mParameterStore.erase(parameterPair);
      if (mTrackedNodes.find(nodeName) != mTrackedNodes.end()) {
        mTrackedNodes[nodeName]--;
        if (mTrackedNodes[nodeName] == 0) {
          mTrackedNodes.erase(nodeName);
        }
      }
    } else {
      warningMessage = "Parameter not tracked";
      return false;
    }
    return true;
  }

  std::string GetParameterType(const ParameterKey &parameterPair, std::string &result, std::string & warningMessage){
  if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
    result = mParameterStore[parameterPair].get_type_name();
  } else {
    warningMessage =  "nodeName : " + parameterPair.first + ":" + parameterPair.second + "is not tracked"; 
    result = ""; //does not exist
  }
  return result;
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
    errorMessage = "Parameter not tracked";
    return false;
}

bool GetParameterAsBool(const ParameterKey & parameterPair, bool & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
        result = mParameterStore[parameterPair].as_bool(); // if not set add another excep
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
    errorMessage = "Parameter not tracked";
    return false;
}

bool GetParameterAsDouble(const ParameterKey & parameterPair, double & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
      result = mParameterStore[parameterPair].as_double();
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
    errorMessage = "Parameter not tracked";
    return false;
}

bool GetParameterAsVectorOfBools(const ParameterKey & parameterPair, std::vector<bool> & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
      result = mParameterStore[parameterPair].as_bool_array();
      } catch (const std::runtime_error & e) {
        errorMessage = "GetParameterAsVectorOfBools caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    errorMessage = "Parameter not tracked";
    return false;
}

bool GetParameterAsVectorOfIntegers(const ParameterKey & parameterPair, std::vector<int64_t> & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
      result = mParameterStore[parameterPair].as_integer_array();
      } catch (const std::runtime_error & e) {
        errorMessage = "GetParameterAsVectorOfIntegers caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    errorMessage = "Parameter not tracked";
    return false;
}

bool GetParameterAsVectorOfDoubles(const ParameterKey & parameterPair, std::vector<double> & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
      result = mParameterStore[parameterPair].as_double_array();
      } catch (const std::runtime_error & e) {
        errorMessage = "GetParameterAsVectorOfDoubles caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    errorMessage = "Parameter not tracked";
    return false;
}

bool GetParameterAsVectorOfStrings(const ParameterKey & parameterPair, std::vector<std::string> & result, std::string & errorMessage) {
    if (mParameterStore.find(parameterPair) != mParameterStore.end()) {
      bool parameterRetrievalStatus = true;
      try {
      result = mParameterStore[parameterPair].as_string_array();
      } catch (const std::runtime_error & e) {
        errorMessage = "GetParameterAsVectorOfStrings caught exception :";
        errorMessage.append(e.what());
        parameterRetrievalStatus = false;
      }
      return parameterRetrievalStatus;
    }
    errorMessage = "Parameter not tracked";
    return false;
}

void listTrackedParameters(){ // rename GetParameterList -> vector<ParameterKeys>
    for (const auto& [key, value] : mParameterStore) {
        std::cerr << "-->" << key.first << ", " << key.second << " -- " << value.value_to_string() << std::endl; 
    }

    std::cerr << "List all tracked nodes " << std::endl;
    std::vector<std::string> nodeList = GetTrackedNodeList();
    for (auto nodeName : nodeList){
      std::cerr << "-->" << nodeName <<std::endl;
    }

    std::cerr << "List all tracked nodes and parameters" << std::endl;
    std::vector<ParameterKey> nodesAndParametersList = GetTrackedNodesAndParametersList();
    for (auto key : nodesAndParametersList){
      std::cerr << "-->" << key.first << " " << key.second <<std::endl;
    }
}

std::vector<std::string> GetTrackedNodeList() {
    std::vector<std::string> nodeList;
    for (const auto& [key, value] : mTrackedNodes) {
        nodeList.push_back(key);
    }
    return nodeList;
}

std::vector<ParameterKey>  GetTrackedNodesAndParametersList() {
  std::vector<ParameterKey> nodesAndParametersList;
  for (const auto& [key, value] : mParameterStore) {
      nodesAndParametersList.push_back(key);
  }
  return nodesAndParametersList;
}

protected:
  vtkMRMLROS2ParameterNode * mMRMLNode;
  std::shared_ptr<rclcpp::ParameterEventHandler> mParameterSubscriber = nullptr;
  std::shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle;
  std::map<ParameterKey , rclcpp::Parameter> mParameterStore; // Parameters  mParameters  this->Parameter  VTK: GetValue()  qt: getValue()
  std::unordered_map<std::string, int> mTrackedNodes; // change to map of counts?
  rclcpp::Parameter mEmptyParameter;
  int mTrackedParametersCount = 0;
  int mAllParametersCount = 0;
  // 
};


#endif
// __vtkMRMLROS2ParameterInternals_h

