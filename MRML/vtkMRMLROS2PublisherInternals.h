#ifndef __vtkMRMLROS2PublisherInternals_h
#define __vtkMRMLROS2PublisherInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

#include <vtkMRMLScene.h>
#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2NodeInternals.h>

class vtkMRMLROS2PublisherInternals
{
public:
  vtkMRMLROS2PublisherInternals(vtkMRMLROS2PublisherNode * mrmlNode):
    mMRMLNode(mrmlNode)
  {}
  virtual ~vtkMRMLROS2PublisherInternals() {};

  virtual bool AddToROS2Node(vtkMRMLScene * scene, const char * nodeId,
			     const std::string & topic, std::string & errorMessage) = 0;
  virtual bool IsAddedToROS2Node(void) const = 0;
  virtual const char * GetROSType(void) const = 0;
  virtual const char * GetSlicerType(void) const = 0;
  virtual size_t GetNumberOfMessagesSent(vtkMRMLScene * scene, const char * nodeId, const std::string & topic) = 0;
  vtkMRMLROS2NODENode * rosNodePtr;

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
  std::shared_ptr<rclcpp::Publisher<_ros_type>> mPublisher = nullptr;

  /**
   * Add the Publisher to the ROS2 node.  This methods searched the
   * vtkMRMLROS2NODENode by Id to locate the rclcpp::node
   */
  bool AddToROS2Node(vtkMRMLScene * scene, const char * nodeId,
		     const std::string & topic, std::string & errorMessage) override
  {
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
    vtkMRMLROS2PublisherNode * pub = rosNodePtr->GetPublisherNodeByTopic(topic);
    if ((pub != nullptr)
	&& pub->IsAddedToROS2Node()) {
      errorMessage = "there is already a publisher for topic \"" + topic + "\" added to the ROS node";
      return false;
    }
    mPublisher = nodePointer->create_publisher<_ros_type>(topic, 10);
    rosNodePtr->SetNthNodeReferenceID("publisher",
				      rosNodePtr->GetNumberOfNodeReferences("publisher"),
				      mMRMLNode->GetID());
    mMRMLNode->SetNodeReferenceID("node", nodeId);
    return true;
  }

  bool IsAddedToROS2Node(void) const override
  {
    return (mPublisher != nullptr);
  }

  const char * GetROSType(void) const override
  {
    return rosidl_generator_traits::name<_ros_type>();
  }

  const char * GetSlicerType(void) const override
  {
    return typeid(_slicer_type).name();
  }

  size_t GetNumberOfMessagesSent(vtkMRMLScene * scene, const char * nodeId, const std::string & topic)
  {
    vtkMRMLNode * rosNodeBasePtr = scene->GetNodeByID(nodeId);
    if (!rosNodeBasePtr) {
      return false;
    }
    vtkMRMLROS2NODENode * rosNodePtr = dynamic_cast<vtkMRMLROS2NODENode *>(rosNodeBasePtr);
    if (!rosNodePtr) {
      return false;
    }
    if (rosNodePtr->GetPublisherNodeByTopic(topic) && rosNodePtr->GetSubscriberNodeByTopic(topic))
    {
      mMRMLNode->mNumberOfMessagesSent++;
    }
    return mMRMLNode->mNumberOfMessagesSent;
  }

};

#endif // __vtkMRMLROS2PublisherInternals_h
