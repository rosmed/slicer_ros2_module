#ifndef __vtkROS2ToSlicer_h
#define __vtkROS2ToSlicer_h

// VTK
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <vtkTable.h>

// ROS2
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

void vtkROS2ToSlicer(const std_msgs::msg::String & input, std::string & result);
void vtkROS2ToSlicer(const std_msgs::msg::Bool & input, bool & result);
void vtkROS2ToSlicer(const sensor_msgs::msg::Joy & input, vtkSmartPointer<vtkTable> result);
void vtkROS2ToSlicer(const geometry_msgs::msg::PoseStamped & input, vtkSmartPointer<vtkMatrix4x4> result);
void vtkROS2ToSlicer(const geometry_msgs::msg::TransformStamped & input, vtkSmartPointer<vtkMatrix4x4> result);

#endif
