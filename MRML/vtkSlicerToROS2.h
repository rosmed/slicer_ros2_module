#ifndef __vtkSlicerToROS2_h
#define __vtkSlicerToROS2_h

// VTK
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

// ROS2
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

void vtkSlicerToROS2(const std::string & input,  std_msgs::msg::String & result);
void vtkSlicerToROS2(const bool & input,  std_msgs::msg::Bool & result);
void vtkSlicerToROS2(vtkMatrix4x4 * input,  geometry_msgs::msg::PoseStamped & result);
void vtkSlicerToROS2(vtkMatrix4x4 * input, geometry_msgs::msg::TransformStamped & result);

// helper function
void vtkMatrix4x4ToQuaternion(vtkMatrix4x4 * input, double quaternion[4]);

#endif
