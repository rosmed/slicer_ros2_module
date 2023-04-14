#ifndef __vtkMRMLROS2NodeInternals_h
#define __vtkMRMLROS2NodeInternals_h

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class vtkMRMLROS2NodeInternals
{
 public:
  std::shared_ptr<rclcpp::Node> mNodePointer;
  std::shared_ptr<tf2_ros::Buffer> mTf2Buffer;
  std::shared_ptr<tf2_ros::TransformListener> mTf2Listener;
};

#endif // __vtkMRMLROS2NodeInternals_h
