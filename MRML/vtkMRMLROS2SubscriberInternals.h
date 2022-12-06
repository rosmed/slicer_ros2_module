#ifndef __vtkMRMLROS2SubscriberInternals_h
#define __vtkMRMLROS2SubscriberInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

#include <vtkMRMLScene.h>

#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2NodeInternals.h>

class vtkMRMLROS2SubscriberInternals
{
public:
  vtkMRMLROS2SubscriberInternals(vtkMRMLROS2SubscriberNode * mrmlNode):
    mMRMLNode(mrmlNode)
  {}
  virtual ~vtkMRMLROS2SubscriberInternals() {};

  virtual bool AddToROS2Node(vtkMRMLScene * scene, const char * nodeId,
			     const std::string & topic, std::string & errorMessage) = 0;
  virtual const char * GetROSType(void) const = 0;
  virtual const char * GetSlicerType(void) const = 0;
  virtual std::string GetLastMessageYAML(void) const = 0;
protected:
  vtkMRMLROS2SubscriberNode * mMRMLNode;
};

template <typename _ros_type, typename _slicer_type>
class vtkMRMLROS2SubscriberTemplatedInternals: public vtkMRMLROS2SubscriberInternals
{
public:
  typedef vtkMRMLROS2SubscriberTemplatedInternals<_ros_type, _slicer_type> SelfType;

  vtkMRMLROS2SubscriberTemplatedInternals(vtkMRMLROS2SubscriberNode *  mrmlNode):
    vtkMRMLROS2SubscriberInternals(mrmlNode)
  {}
  

protected:
  _ros_type mLastMessageROS;
  std::shared_ptr<rclcpp::Subscription<_ros_type>> mSubscription;

  /**
   * This is the ROS callback for the subscription.  This methods
   * saves the ROS message as-is and set the modified flag for the
   * MRML node
   */
  void SubscriberCallback(const _ros_type & message) {
    // \todo is there a timestamp in MRML nodes we can update from the ROS message?
    mLastMessageROS = message;
    mMRMLNode->mNumberOfMessages++;
    mMRMLNode->Modified();
  }

  /**
   * Add the subscriber to the ROS2 node.  This methods searched the
   * vtkMRMLROS2NODENode by Id to locate the rclcpp::node
   */
  bool AddToROS2Node(vtkMRMLScene * scene, const char * nodeId,
		     const std::string & topic, std::string & errorMessage) {
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
    // Check if the subscriber already exists
    vtkMRMLNode * sub = rosNodePtr->GetSubscriberNodeByTopic(topic);
    // if it doesn't instantiate the subscriber
    if (sub == nullptr){
      mSubscription = nodePointer->create_subscription<_ros_type>(topic, 100,
                  std::bind(&SelfType::SubscriberCallback, this, std::placeholders::_1));
      rosNodePtr->SetNthNodeReferenceID("subscriber",
                rosNodePtr->GetNumberOfNodeReferences("subscriber"),
                mMRMLNode->GetID());
      mMRMLNode->SetNodeReferenceID("node", nodeId);
    }
    // Otherwise state that there is already a subscriber for that topic 
    else{
      std::cerr << "Subscriber already added for that topic" << std::endl;
    }
    return true;
  }

  const char * GetROSType(void) const override
  {
    return rosidl_generator_traits::name<_ros_type>();
    // return typeid(_ros_type).name();
  }

  const char * GetSlicerType(void) const override
  {
    return typeid(_slicer_type).name();
  }

  std::string GetLastMessageYAML(void) const override
  {
    std::stringstream out;
    rosidl_generator_traits::to_yaml(mLastMessageROS, out);
    return out.str();
  }
};

#endif // __vtkMRMLROS2SubscriberInternals_h
