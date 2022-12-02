#ifndef __vtkMRMLROS2PublisherInternals_h
#define __vtkMRMLROS2PublisherInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

#include <vtkMRMLScene.h>
#include <chrono>
#include <functional>
#include <memory>
#include <std_msgs/msg/string.hpp>

#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2NodeInternals.h>

using namespace std::chrono_literals;

class vtkMRMLROS2PublisherInternals
{
public:
  vtkMRMLROS2PublisherInternals(vtkMRMLROS2PublisherNode * mrmlNode):
    mMRMLNode(mrmlNode)
  {}
  virtual ~vtkMRMLROS2PublisherInternals() {};

  virtual bool AddToROS2Node(vtkMRMLScene * scene, const char * nodeId,
			     const std::string & topic, std::string & errorMessage) = 0;
  virtual const char * GetROSType(void) const = 0;
  virtual const char * GetSlicerType(void) const = 0;
  // virtual std::string GetLastMessageYAML(void) const = 0;
  vtkMRMLROS2NODENode * rosNodePtr;
  int nthRef = 0;

protected:
  vtkMRMLROS2PublisherNode * mMRMLNode;
};

template <typename _slicer_type, typename _ros_type>
class vtkMRMLROS2PublisherTemplatedInternals: public vtkMRMLROS2PublisherInternals
{
public:
  typedef vtkMRMLROS2PublisherTemplatedInternals<_slicer_type, _ros_type> SelfType;

  vtkMRMLROS2PublisherTemplatedInternals(vtkMRMLROS2PublisherNode *  mrmlNode):
    vtkMRMLROS2PublisherInternals(mrmlNode)
  {}

protected:
  _ros_type mMessageROS;
  std::shared_ptr<rclcpp::Publisher<_ros_type>> mPublisher;

  /**
   * Add the Publisher to the ROS2 node.  This methods searched the
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
    mPublisher = nodePointer->create_publisher<_ros_type>(topic, 10);
    rosNodePtr->SetNthNodeReferenceID("publisher",
				      rosNodePtr->GetNumberOfNodeReferences("publisher"),
				      mMRMLNode->GetID());
    mMRMLNode->SetNodeReferenceID("node", nodeId);
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

};

#endif // __vtkMRMLROS2PublisherInternals_h
