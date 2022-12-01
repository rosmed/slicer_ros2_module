#ifndef __vtkMRMLROS2PublisherInternals_h
#define __vtkMRMLROS2PublisherInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

#include <vtkMRMLScene.h>
#include <chrono>
#include <functional>
#include <memory>
#include "std_msgs/msg/string.hpp"

#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2NodeInternals.h>

using namespace std::chrono_literals;

class vtkMRMLROS2PublisherInternals
{
public:
  vtkMRMLROS2PublisherInternals(vtkMRMLROS2PublisherNode * mrmlNode):
    mMRMLNode(mrmlNode)
  {}

  virtual bool AddToROS2Node(vtkMRMLScene * scene, const char * nodeId,
			     const std::string & topic, std::string & errorMessage) = 0;
  virtual const char * GetROSType(void) const = 0;
  virtual const char * GetSlicerType(void) const = 0;
  virtual std::string GetLastMessageYAML(void) const = 0;
  vtkMRMLROS2NodeNode * rosNodePtr;
  int nthRef = 0;

protected:
  vtkMRMLROS2PublisherNode * mMRMLNode;
};

template <typename _ros_type, typename _slicer_type>
class vtkMRMLROS2PublisherTemplatedInternals: public vtkMRMLROS2PublisherInternals
{
public:
  typedef vtkMRMLROS2PublisherTemplatedInternals<_ros_type, _slicer_type> SelfType;

  vtkMRMLROS2PublisherTemplatedInternals(vtkMRMLROS2PublisherNode *  mrmlNode):
    vtkMRMLROS2PublisherInternals(mrmlNode)
  {}

protected:
  _ros_type mLastMessageROS;
  std::shared_ptr<rclcpp::Publisher<_ros_type>> mPublisher;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  /**
   * This is the ROS callback for the subscription.  This methods
   * saves the ROS message as-is and set the modified flag for the
   * MRML node
   */
  void PublisherCallback() {
    // \todo is there a timestamp in MRML nodes we can update from the ROS message?
    auto message = std_msgs::msg::String(); // typecasted to string - switch to templated
    message.data = "Hello, world! " + std::to_string(count_++);
    mPublisher->publish(message);
    rosNodePtr->Modified();
    // mLastMessageROS = message;
    // mMRMLNode->mNumberOfMessages++;
    // mMRMLNode->Modified(); // Should just be able to call this but it's not working
    // rosNodePtr->Modified();
  }

  /**
   * Add the Publisher to the ROS2 node.  This methods searched the
   * vtkMRMLROS2NodeNode by Id to locate the rclcpp::node
   */
  bool AddToROS2Node(vtkMRMLScene * scene, const char * nodeId,
		     const std::string & topic, std::string & errorMessage) {
    vtkMRMLNode * rosNodeBasePtr = scene->GetNodeByID(nodeId);
    if (!rosNodeBasePtr) {
      errorMessage = "unable to locate node";
      return false;
    }
    // vtkMRMLROS2NodeNode * rosNodePtr = dynamic_cast<vtkMRMLROS2NodeNode *>(rosNodeBasePtr);
    rosNodePtr = dynamic_cast<vtkMRMLROS2NodeNode *>(rosNodeBasePtr);
    if (!rosNodePtr) {
      errorMessage = std::string(rosNodeBasePtr->GetName()) + " doesn't seem to be a vtkMRMLROS2NodeNode";
      return false;
    }
    std::shared_ptr<rclcpp::Node> nodePointer = rosNodePtr->mInternals->mNodePointer;
    mPublisher = nodePointer->create_publisher<_ros_type>("slicer_sub", 10);
    timer_ = nodePointer->create_wall_timer(500ms, std::bind(&SelfType::PublisherCallback, this));
    rosNodePtr->SetAndObserveNthNodeReferenceID(topic.c_str(), nthRef, mMRMLNode->GetID()); // Set up node references
    nthRef++;
    rosNodePtr->Modified();
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

#endif // __vtkMRMLROS2PublisherInternals_h
