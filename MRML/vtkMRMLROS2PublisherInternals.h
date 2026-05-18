#ifndef __vtkMRMLROS2PublisherInternals_h
#define __vtkMRMLROS2PublisherInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

#include <vtkMRMLScene.h>
#include <vtkMRMLROS2Utils.h>
#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2NodeInternals.h>
#include <type_traits>

namespace SlicerROS2Internal {
  template <typename T, typename = void>
  struct has_header : std::false_type {};

  template <typename T>
  struct has_header<T, std::void_t<decltype(std::declval<T>().header.frame_id)>> : std::true_type {};

  template <typename T>
  void SetFrameId(T & msg, const std::string & frameId) {
    if constexpr (has_header<T>::value) {
      if (!frameId.empty()) {
        msg.header.frame_id = frameId;
      }
    }
  }
}

class vtkMRMLROS2PublisherInternals
{
public:
  vtkMRMLROS2PublisherInternals(vtkMRMLROS2PublisherNode * mrmlNode):
    mMRMLNode(mrmlNode)
  {}
  virtual ~vtkMRMLROS2PublisherInternals() = default;

  virtual bool AddToROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                             const std::string & topic, std::string & errorMessage) = 0;
  virtual bool RemoveFromROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                                  const std::string & topic, std::string & errorMessage) = 0;
  virtual bool IsAddedToROS2Node(void) const = 0;
  virtual const char * GetROSType(void) const = 0;
  virtual const char * GetSlicerType(void) const = 0;
  virtual std::shared_ptr<rclcpp::Node> GetROSNode() const = 0;
protected:
  vtkMRMLROS2PublisherNode * mMRMLNode;
  std::shared_ptr<rclcpp::Node> mROSNode = nullptr;
};



template <typename _slicer_type, typename _ros_type>
class vtkMRMLROS2PublisherTemplatedInternals: public vtkMRMLROS2PublisherInternals
{
public:
  typedef vtkMRMLROS2PublisherTemplatedInternals<_slicer_type, _ros_type> SelfType;

  vtkMRMLROS2PublisherTemplatedInternals(vtkMRMLROS2PublisherNode *  mrmlNode):
    vtkMRMLROS2PublisherInternals(mrmlNode)
  {}

  size_t PublishDirect(const _ros_type & rosMessage)
  {
    if (!mPublisher) return 0;
    const auto nbSubscriber = mPublisher->get_subscription_count();
    if (nbSubscriber != 0) {
      mPublisher->publish(rosMessage);
    }
    return nbSubscriber;
  }

  std::shared_ptr<rclcpp::Node> GetROSNode() const override
  {
    return mROSNode;
  }

protected:
  std::shared_ptr<rclcpp::Publisher<_ros_type>> mPublisher = nullptr;

  /**
   * Add the Publisher to the ROS2 node.  This methods searched the
   * vtkMRMLROS2NodeNode by Id to locate the rclcpp::node
   */
  bool AddToROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                     const std::string & topic, std::string & errorMessage) override
  {
    vtkMRMLROS2NodeNode * mrmlROSNodePtr = vtkMRMLROS2::CheckROS2NodeExists(nodeInScene, nodeId, errorMessage);
    if (!mrmlROSNodePtr) return false;

    vtkMRMLROS2PublisherNode * pub = mrmlROSNodePtr->GetPublisherNodeByTopic(topic);
    if ((pub != nullptr)
        && pub->IsAddedToROS2Node()) {
      errorMessage = "there is already a publisher for topic \"" + topic + "\" added to the ROS node";
      return false;
    }
    mROSNode = mrmlROSNodePtr->mInternals->mNodePointer;

    rclcpp::QoS qos_profile(mMRMLNode->GetQoSHistoryDepth());
    switch (mMRMLNode->GetQoSReliability()) {
      case vtkMRMLROS2PublisherNode::Reliable:
        qos_profile.reliable();
        break;
      case vtkMRMLROS2PublisherNode::BestEffort:
        qos_profile.best_effort();
        break;
      default:
        break;
    }
    switch (mMRMLNode->GetQoSDurability()) {
      case vtkMRMLROS2PublisherNode::TransientLocal:
        qos_profile.transient_local();
        break;
      case vtkMRMLROS2PublisherNode::Volatile:
        qos_profile.durability_volatile();
        break;
      default:
        break;
    }

    mPublisher = mROSNode->create_publisher<_ros_type>(topic, qos_profile);
    mrmlROSNodePtr->SetNthNodeReferenceID("publisher",
                                          mrmlROSNodePtr->GetNumberOfNodeReferences("publisher"),
                                          mMRMLNode->GetID());
    mMRMLNode->SetNodeReferenceID("node", nodeId);
    mrmlROSNodePtr->WarnIfNotSpinning("adding publisher for \"" + topic + "\"");
    return true;
  }

  bool RemoveFromROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                          const std::string & topic, std::string & errorMessage) override
  {
    vtkMRMLROS2NodeNode * rosNodePtr = vtkMRMLROS2::CheckROS2NodeExists(nodeInScene, nodeId, errorMessage);
    if (!rosNodePtr) return false;

    vtkMRMLROS2PublisherNode * pub = rosNodePtr->GetPublisherNodeByTopic(topic);
    if (pub == nullptr || !pub->IsAddedToROS2Node()) {
      errorMessage = "there isn't a publisher for topic \"" + topic + "\" which can be deleted from the ROS node";
      return false;
    }

    mMRMLNode->SetNodeReferenceID("node", nullptr);
    rosNodePtr->RemoveNthNodeReferenceID("publisher",
                                         rosNodePtr->GetNumberOfNodeReferences("publisher"));

    mPublisher.reset();
    mROSNode.reset();

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
};



template <typename _slicer_type, typename _ros_type>
class vtkMRMLROS2PublisherNativeInternals:
  public vtkMRMLROS2PublisherTemplatedInternals<_slicer_type, _ros_type>
{
public:
  typedef vtkMRMLROS2PublisherTemplatedInternals<_slicer_type, _ros_type> BaseType;

  vtkMRMLROS2PublisherNativeInternals(vtkMRMLROS2PublisherNode * mrmlNode):
    BaseType(mrmlNode)
  {}

  _slicer_type mLastMessageSlicer;

  size_t Publish(const _slicer_type & message)
  {
    const auto nbSubscriber = this->mPublisher->get_subscription_count();
    if (nbSubscriber != 0) {
      _ros_type rosMessage;
      vtkSlicerToROS2(message, rosMessage, BaseType::mROSNode);
      SlicerROS2Internal::SetFrameId(rosMessage, this->mMRMLNode->GetFrameId());
      this->mPublisher->publish(rosMessage);
    }
    return nbSubscriber;
  }
};



template <typename _slicer_type, typename _ros_type>
class vtkMRMLROS2PublisherVTKInternals:
  public vtkMRMLROS2PublisherTemplatedInternals< _slicer_type, _ros_type>
{
public:
  typedef vtkMRMLROS2PublisherTemplatedInternals< _slicer_type, _ros_type> BaseType;

  vtkMRMLROS2PublisherVTKInternals(vtkMRMLROS2PublisherNode * mrmlNode):
    BaseType(mrmlNode)
  {
    mLastMessageSlicer = vtkNew<_slicer_type>();
  }

  vtkSmartPointer<_slicer_type> mLastMessageSlicer;

  size_t Publish(_slicer_type * message)
  {
    const auto nbSubscriber = this->mPublisher->get_subscription_count();
    if (nbSubscriber != 0) {
      _ros_type rosMessage;
      vtkSlicerToROS2(message, rosMessage, BaseType::mROSNode);
      SlicerROS2Internal::SetFrameId(rosMessage, this->mMRMLNode->GetFrameId());
      this->mPublisher->publish(rosMessage);
    }
    return nbSubscriber;
  }
};

#endif // __vtkMRMLROS2PublisherInternals_h
