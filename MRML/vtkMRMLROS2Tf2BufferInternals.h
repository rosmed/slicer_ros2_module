#ifndef __vtkMRMLROS2Tf2BufferInternals_h
#define __vtkMRMLROS2Tf2BufferInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class vtkMRMLROS2Tf2BufferInternals
{
 public:

  virtual ~vtkMRMLROS2Tf2BufferInternals() = default;
  std::shared_ptr<tf2_ros::Buffer> mTfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> mTfListener;
  std::shared_ptr<rclcpp::Node> mNodePointer;
};

#endif // __vtkMRMLROS2Tf2BufferInternals_h