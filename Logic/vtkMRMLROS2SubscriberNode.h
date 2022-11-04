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

  inline const char* GetNodeTagName() override { return mTopic.c_str(); };

  void SetTopic(const std::string & topic);
  size_t GetNumberOfMessages(void) const;

//  virtual vtkObject * GetLastMessage(void) const = 0;
  virtual std::string GetLastMessageYAML(void) const = 0; // Conclusions - virtual functions are causing new error?? Not being linked properly (location??)

 protected:
  //----------------------------------------------------------------
  // Constructor and destructor
  //----------------------------------------------------------------

  vtkMRMLROS2SubscriberNode();
  ~vtkMRMLROS2SubscriberNode();

  std::string mTopic = "undefined";
  size_t mNumberOfMessages = 0;

 public:
  virtual void SetSubscriber(std::shared_ptr<rclcpp::Node> mNodePointer) = 0;
};

#endif
