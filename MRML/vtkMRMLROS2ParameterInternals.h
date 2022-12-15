#ifndef __vtkMRMLROS2ParameterInternals_h
#define __vtkMRMLROS2ParameterInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

#include <vtkMRMLScene.h>
#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2NodeInternals.h>

class vtkMRMLROS2ParameterInternals
{
public:
  vtkMRMLROS2ParameterInternals(vtkMRMLROS2ParameterNode * mrmlNode):
    mMRMLNode(mrmlNode)
  {}
  // virtual ~vtkMRMLROS2ParameterInternals() = default;

  /**
   * Add the subscriber to the ROS2 node.  This methods searched the
   * vtkMRMLROS2NODENode by Id to locate the rclcpp::node
   */
  bool AddToROS2Node(vtkMRMLScene * scene, const char * nodeId,
		     const std::string & trackedNodeName, std::string & errorMessage) {

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

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(nodePointer);

    auto cb = [trackedNodeName, nodePointer](const rcl_interfaces::msg::ParameterEvent &event)
    {
        // Obtain list of parameters that changed
        std::cerr << "Event node :"<< event.node.c_str() << std::endl;
        if(!strcmp(event.node.c_str(),trackedNodeName.c_str())){
          auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);

          // Iterate through every parameter in the list
          for (auto &p : params)
          {
              // Create a string message from the info received.
              std::string msgString = "Param Name :" + p.get_name() + " | Param Type : " + p.get_type_name() + " | Param Value : " + p.value_to_string();
              std::cerr << "Received an update to a parameter"  << std::endl;
              std::cerr << msgString << "\n" <<std::endl;
          }
        }
    };

    cb_handle = param_subscriber_->add_parameter_event_callback(cb);

    rosNodePtr->SetNthNodeReferenceID("parameter",
				      rosNodePtr->GetNumberOfNodeReferences("parameter"),
				      mMRMLNode->GetID());
              
    mMRMLNode->SetNodeReferenceID("node", nodeId);

    return true;

  }

  bool IsAddedToROS2Node(void) const
  {
    return (param_subscriber_ != nullptr);
  }

protected:
  vtkMRMLROS2ParameterNode * mMRMLNode;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_ = nullptr;
  std::shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle;
};


#endif
// __vtkMRMLROS2ParameterInternals_h

