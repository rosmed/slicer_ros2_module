#ifndef __vtkMRMLROS2SubscriberImplementation_h
#define __vtkMRMLROS2SubscriberImplementation_h

#include <vtkMRMLROS2SubscriberNode.h>
#include <vtkSlicerRos2ModuleLogicExport.h>

template <typename _ros_type, typename _slicer_type>
class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkMRMLROS2SubscriberImplementation: public vtkMRMLROS2SubscriberNode
{
 private:
  _ros_type mLastMessage;
  std::shared_ptr<rclcpp::Subscription<_ros_type>> mSubscription;

 protected:

  vtkMRMLROS2SubscriberImplementation() {};
  ~vtkMRMLROS2SubscriberImplementation() {};

  /**
   * This is the ROS callback for the subscription.  This methods
   * saves the ROS message as-is and set the modified flag for the
   * MRML node
   */
  void SubscriberCallback(const _ros_type & message) {
    // \todo is there a timestamp in MRML nodes we can update from the ROS message?
    mLastMessage = message;
    mNumberOfMessages++;
    this->Modified();
  }

 public:
  typedef vtkMRMLROS2SubscriberImplementation<_ros_type, _slicer_type> SelfType;
  vtkTemplateTypeMacro(SelfType, vtkMRMLROS2SubscriberNode);
  _ros_type type;
  _slicer_type slicerType;
  static SelfType * New(void); // vtkObject - Create an object with Debug turned off, modified time initialized to zero, and reference counting on.

  // Create instance of the default node. Like New only virtual. -> Explanation from doxygen
  virtual vtkMRMLNode * CreateNodeInstance(void) override {
    return SelfType::New();
  };

  void PrintSelf(ostream&, vtkIndent) override {};

  // todo this method should not require a direct pointer on the
  // rclcpp::Node but shoudl use a MRMLROS2NodeNode, either by name or
  // vtk pointer, dynamic cast and then retrieve the rclcpp::Node -
  // this will require to add some friend declarations
  void SetSubscriber(std::shared_ptr<rclcpp::Node> nodePointer) {
    std::cerr << "SetSubscriber for " << mTopic << std::endl;
    mSubscription= nodePointer->create_subscription<_ros_type>(mTopic, 10000,
							       std::bind(&SelfType::SubscriberCallback, this, std::placeholders::_1));
  }

  /**
   * Get the latest message received by the subscriber.  The message
   * is converted from the ROS type to a Slicer type by the overloaded
   * global function vtkROS2ToSlicer.
   */
  void GetLastMessage(_slicer_type & result) const
  {
    // todo maybe add some check that we actually received a message?
    vtkROS2ToSlicer(mLastMessage, result);
    // make sure the result is actually stored in the result
  }

  /**
   * Retrieve the last message in YAML format, this can be used in
   * Python to get all the information received.  The main drawbacks
   * are that it involves string parsing which takes time and doesn't
   * provide the full resolution for floating point numbers.
   */
  std::string GetLastMessageYAML(void) const override
  {
    std::stringstream out;
    rosidl_generator_traits::to_yaml(mLastMessage, out);
    return out.str();
  }
};

#endif
