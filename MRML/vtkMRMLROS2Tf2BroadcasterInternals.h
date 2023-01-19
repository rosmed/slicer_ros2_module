#ifndef __vtkMRMLROS2Tf2BroadcasterInternals_h
#define __vtkMRMLROS2Tf2BroadcasterInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

// SlicerROS2 includes
#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2NodeInternals.h>

// Slicer includes
#include <vtkMRMLScene.h>
#include <vtkSlicerToROS2.h>
#include <vtkMRMLTransformNode.h> // should this go here 
#include <vtkMatrix4x4.h>

class vtkMRMLROS2Tf2BroadcasterInternals
{

 public:

  virtual ~vtkMRMLROS2Tf2BroadcasterInternals() = default;
  std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
  std::shared_ptr<rclcpp::Node> mNodePointer;
};

#endif // __vtkMRMLROS2Tf2BroadcasterInternals_h
