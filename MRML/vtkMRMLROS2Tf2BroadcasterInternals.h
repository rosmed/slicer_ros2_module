#ifndef __vtkMRMLROS2Tf2BroadcasterInternals_h
#define __vtkMRMLROS2Tf2BroadcasterInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class vtkMRMLROS2Tf2BroadcasterInternals
{
  friend class vtkMRMLROS2Tf2BroadcasterNode;
public:
  virtual ~vtkMRMLROS2Tf2BroadcasterInternals() = default;
protected:
  std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
  std::shared_ptr<rclcpp::Node> mROSNode = nullptr;
};

#endif // __vtkMRMLROS2Tf2BroadcasterInternals_h
