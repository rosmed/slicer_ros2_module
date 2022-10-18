#ifndef __vtkMRMLROS2SubscriberNode_h
#define __vtkMRMLROS2SubscriberNode_h
// MRML includes
#include <vtkMRML.h>
#include <vtkMRMLNode.h>
#include <vtkMRMLStorageNode.h>
#include <vtkMRMLScene.h>



// VTK includes
#include <vtkObject.h>

// STD includes
#include <string>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "vtkSlicerRos2ModuleLogicExport.h"



class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkMRMLROS2SubscriberNode : public vtkMRMLNode
{

public:

  static vtkMRMLROS2SubscriberNode *New();
  vtkTypeMacro(vtkMRMLROS2SubscriberNode, vtkMRMLNode);

  void PrintSelf(ostream& os, vtkIndent indent) override{};
  inline const char* GetNodeTagName() override { return "SlicerROS2Module"; };
  virtual vtkMRMLNode* CreateNodeInstance() override;


  void SubscriberCallBack(const geometry_msgs::msg::PoseStamped& pose);




protected:
  //----------------------------------------------------------------
  // Constructor and destroctor
  //----------------------------------------------------------------

  vtkMRMLROS2SubscriberNode();
  ~vtkMRMLROS2SubscriberNode();


public:
  //----------------------------------------------------------------
  // Connector configuration
  //----------------------------------------------------------------
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> mSubscription;
  void SetSubscriber(std::shared_ptr<rclcpp::Node> mNodePointer);

private:
  //----------------------------------------------------------------
  // Data
  //----------------------------------------------------------------

};

#endif
