#ifndef __vtkROS2ToSlicer_h
#define __vtkROS2ToSlicer_h

// VTK
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <vtkTable.h>
#include <vtkIntArray.h>
#include <vtkDoubleArray.h>
#include <vtkTable.h>

// ROS2
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

void vtkROS2ToSlicer(const std_msgs::msg::String & input, std::string & result);
void vtkROS2ToSlicer(const std_msgs::msg::Bool & input, bool & result);
void vtkROS2ToSlicer(const std_msgs::msg::Int64 & input, int & result);
void vtkROS2ToSlicer(const std_msgs::msg::Float64 & input, double & result);
void vtkROS2ToSlicer(const std_msgs::msg::Int64MultiArray & input, vtkSmartPointer<vtkIntArray> result);
void vtkROS2ToSlicer(const std_msgs::msg::Float64MultiArray & input, vtkSmartPointer<vtkDoubleArray> result);

void vtkROS2ToSlicer(const std_msgs::msg::Int64MultiArray & input, vtkSmartPointer<vtkTable> result);
void vtkROS2ToSlicer(const std_msgs::msg::Float64MultiArray & input, vtkSmartPointer<vtkTable> result);
void vtkROS2ToSlicer(const sensor_msgs::msg::Joy & input, vtkSmartPointer<vtkTable> result);
void vtkROS2ToSlicer(const geometry_msgs::msg::PoseStamped & input, vtkSmartPointer<vtkMatrix4x4> result);
void vtkROS2ToSlicer(const geometry_msgs::msg::TransformStamped & input, vtkSmartPointer<vtkMatrix4x4> result);
void vtkROS2ToSlicer(const std_srvs::srv::Trigger::Response & input, vtkSmartPointer<vtkTable> result);

#endif
