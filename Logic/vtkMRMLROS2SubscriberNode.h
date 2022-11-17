#ifndef __vtkMRMLROS2SubscriberNode_h
#define __vtkMRMLROS2SubscriberNode_h

// MRML includes
#include <vtkMRMLNode.h>

// STD includes
#include <string>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

#include "vtkSlicerRos2ModuleLogicExport.h"

class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkMRMLROS2SubscriberNode : public vtkMRMLNode
{
 public:
  vtkTypeMacro(vtkMRMLROS2SubscriberNode, vtkMRMLNode);

  inline const char * GetNodeTagName(void) override {
    return "ROS2Subscriber"; };

  void SetTopic(const std::string & topic);

  const char * GetTopic(void) const {
    return mTopic.c_str();
  }

  size_t GetNumberOfMessages(void) const;

  virtual const char * GetROSType(void) const = 0;

  virtual const char * GetSlicerType(void) const = 0;

  /**
   * Get the latest ROS message in YAML format
   */
  virtual std::string GetLastMessageYAML(void) const = 0;

  /**
   * Get the latest message as a vtkVariant.  This method will use the
   * latest ROS message received and convert it to the internal
   * Slicer/VTK type if needed.  The result of the conversion is
   * cached so future calls to GetLastMessage don't require converting
   * again
   */
  virtual vtkVariant GetLastMessageVariant(void) = 0;

 protected:
  //----------------------------------------------------------------
  // Constructor and destructor
  //----------------------------------------------------------------

  vtkMRMLROS2SubscriberNode();
  ~vtkMRMLROS2SubscriberNode();

  std::string mTopic = "undefined";
  std::string mNodeName = "ros2:undefined";
  size_t mNumberOfMessages = 0;

 public:
  virtual void SetSubscriber(std::shared_ptr<rclcpp::Node> mNodePointer) = 0;
};

#endif
