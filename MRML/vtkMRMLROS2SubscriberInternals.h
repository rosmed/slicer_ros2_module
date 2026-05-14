#ifndef __vtkMRMLROS2SubscriberInternals_h
#define __vtkMRMLROS2SubscriberInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

#include <vtkMRMLScene.h>
#include <vtkMRMLROS2Utils.h>
#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2NodeInternals.h>

class vtkMRMLROS2SubscriberInternals
{
public:
  vtkMRMLROS2SubscriberInternals(vtkMRMLROS2SubscriberNode * mrmlNode):
    mMRMLNode(mrmlNode)
  {}
  virtual ~vtkMRMLROS2SubscriberInternals() = default;

  virtual bool AddToROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                             const std::string & topic, std::string & errorMessage) = 0;
  virtual bool RemoveFromROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                                  const std::string & topic, std::string & errorMessage) = 0;
  virtual bool IsAddedToROS2Node(void) const = 0;
  virtual const char * GetROSType(void) const = 0;
  virtual const char * GetSlicerType(void) const = 0;
  virtual std::string GetLastMessageYAML(void) const = 0;
protected:
  vtkMRMLROS2SubscriberNode * mMRMLNode;
  std::shared_ptr<rclcpp::Node> mROSNode = nullptr;
};



template <typename _ros_type, typename _slicer_type>
class vtkMRMLROS2SubscriberTemplatedInternals: public vtkMRMLROS2SubscriberInternals
{
public:
  typedef vtkMRMLROS2SubscriberTemplatedInternals<_ros_type, _slicer_type> SelfType;

  vtkMRMLROS2SubscriberTemplatedInternals(vtkMRMLROS2SubscriberNode *  mrmlNode):
    vtkMRMLROS2SubscriberInternals(mrmlNode)
  {}

  ~vtkMRMLROS2SubscriberTemplatedInternals()
  {}

protected:
  _ros_type mLastMessageROS;
  std::shared_ptr<rclcpp::Subscription<_ros_type>> mSubscription = nullptr;

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
   * vtkMRMLROS2NodeNode by Id to locate the rclcpp::node
   */
  bool AddToROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                     const std::string & topic, std::string & errorMessage) {
    vtkMRMLROS2NodeNode * mrmlROSNodePtr = vtkMRMLROS2::CheckROS2NodeExists(nodeInScene, nodeId, errorMessage);
    if (!mrmlROSNodePtr) return false;

    vtkMRMLROS2SubscriberNode * sub = mrmlROSNodePtr->GetSubscriberNodeByTopic(topic);
    if ((sub != nullptr) && sub->IsAddedToROS2Node()) {
      errorMessage = "there is already a subscriber for topic \"" + topic + "\" added to the ROS node";
      return false;
    }
    mROSNode = mrmlROSNodePtr->mInternals->mNodePointer;

    rclcpp::QoS qos_profile(mMRMLNode->GetQoSHistoryDepth());
    switch (mMRMLNode->GetQoSReliability()) {
      case vtkMRMLROS2SubscriberNode::Reliable:
        qos_profile.reliable();
        break;
      case vtkMRMLROS2SubscriberNode::BestEffort:
        qos_profile.best_effort();
        break;
      default:
        break;
    }
    switch (mMRMLNode->GetQoSDurability()) {
      case vtkMRMLROS2SubscriberNode::TransientLocal:
        qos_profile.transient_local();
        break;
      case vtkMRMLROS2SubscriberNode::Volatile:
        qos_profile.durability_volatile();
        break;
      default:
        break;
    }

    mSubscription
      = mROSNode->create_subscription<_ros_type>(topic, qos_profile,
                                                 std::bind(&SelfType::SubscriberCallback, this, std::placeholders::_1));
    mrmlROSNodePtr->SetNthNodeReferenceID("subscriber",
                                          mrmlROSNodePtr->GetNumberOfNodeReferences("subscriber"),
                                          mMRMLNode->GetID());
    mMRMLNode->SetNodeReferenceID("node", nodeId);
    mrmlROSNodePtr->WarnIfNotSpinning("adding subscriber for \"" + topic + "\"");
    return true;
  }

  bool RemoveFromROS2Node(vtkMRMLNode * nodeInScene, const char * nodeId,
                          const std::string & topic, std::string & errorMessage) override
  {
    vtkMRMLROS2NodeNode * rosNodePtr = vtkMRMLROS2::CheckROS2NodeExists(nodeInScene, nodeId, errorMessage);
    if(!rosNodePtr) return false;

    vtkMRMLROS2SubscriberNode * sub = rosNodePtr->GetSubscriberNodeByTopic(topic);
    if (sub == nullptr || !sub->IsAddedToROS2Node()) {
      errorMessage = "there isn't a subscriber for topic \"" + topic + "\" which can be deleted from the ROS node";
      return false;
    }

    mMRMLNode->SetNodeReferenceID("node", nullptr);
    rosNodePtr->RemoveNthNodeReferenceID("subscriber",
                                         rosNodePtr->GetNumberOfNodeReferences("subscriber"));

    mSubscription.reset();
    mROSNode.reset();

    return true;
  }

  bool IsAddedToROS2Node(void) const override
  {
    return (mSubscription != nullptr);
  }

  const char * GetROSType(void) const override
  {
    return rosidl_generator_traits::name<_ros_type>();
  }

  const char * GetSlicerType(void) const override
  {
    return typeid(_slicer_type).name();
  }

  std::string GetLastMessageYAML(void) const override
  {
    std::stringstream out;
    to_block_style_yaml(mLastMessageROS, out);
    return out.str();
  }
};



template <typename _ros_type, typename _slicer_type>
class vtkMRMLROS2SubscriberNativeInternals:
  public vtkMRMLROS2SubscriberTemplatedInternals<_ros_type, _slicer_type>
{
public:
  typedef vtkMRMLROS2SubscriberTemplatedInternals<_ros_type, _slicer_type> BaseType;

  vtkMRMLROS2SubscriberNativeInternals(vtkMRMLROS2SubscriberNode * mrmlNode):
    BaseType(mrmlNode)
  {}

  _slicer_type mLastMessageSlicer;

  void GetLastMessage(_slicer_type & result)
  {
    // todo maybe add some check that we actually received a message?
    vtkROS2ToSlicer(this->mLastMessageROS, result);
  }

  _slicer_type GetLastMessage(void)
  {
    this->GetLastMessage(mLastMessageSlicer);
    return mLastMessageSlicer;
  }

  vtkVariant GetLastMessageVariant(void)
  {
    this->GetLastMessage(mLastMessageSlicer);
    return vtkVariant(mLastMessageSlicer);
  }
};



template <typename _ros_type, typename _slicer_type>
class vtkMRMLROS2SubscriberVTKInternals:
  public vtkMRMLROS2SubscriberTemplatedInternals<_ros_type, _slicer_type>
{
public:
  typedef vtkMRMLROS2SubscriberTemplatedInternals<_ros_type, _slicer_type> BaseType;

  vtkMRMLROS2SubscriberVTKInternals(vtkMRMLROS2SubscriberNode * mrmlNode):
    BaseType(mrmlNode)
  {
    mLastMessageSlicer = vtkNew<_slicer_type>();
  }

  vtkSmartPointer<_slicer_type> mLastMessageSlicer;

  void GetLastMessage(_slicer_type * result)
  {
    // todo maybe add some check that we actually received a message?
    vtkROS2ToSlicer(this->mLastMessageROS, result);
  }

  _slicer_type * GetLastMessage(void)
  {
    this->GetLastMessage(mLastMessageSlicer.GetPointer());
    return mLastMessageSlicer.GetPointer();
  }

  vtkVariant GetLastMessageVariant(void)
  {
    this->GetLastMessage(mLastMessageSlicer.GetPointer());
    return vtkVariant(mLastMessageSlicer.GetPointer());
  }
};

#endif // __vtkMRMLROS2SubscriberInternals_h
