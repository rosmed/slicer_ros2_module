#ifndef __vtkROS2ToSlicer_h
#define __vtkROS2ToSlicer_h

// VTK
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <vtkTable.h>
#include <vtkIntArray.h>
#include <vtkDoubleArray.h>
#include <vtkTable.h>
#include <vtkTypeUInt8Array.h>
#include <vtkPoints.h>

// ROS2
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/transform.hpp"
#include <geometry_msgs/msg/wrench.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

// custom types
#include <vtkCustomTypes.h>

// std_msgs
void vtkROS2ToSlicer(const std_msgs::msg::String & input, std::string & result);
void vtkROS2ToSlicer(const std_msgs::msg::Bool & input, bool & result);
void vtkROS2ToSlicer(const std_msgs::msg::Int64 & input, int & result);
void vtkROS2ToSlicer(const std_msgs::msg::Float64 & input, double & result);

// for vectors
void vtkROS2ToSlicer(const std_msgs::msg::Int64MultiArray & input, vtkSmartPointer<vtkIntArray> result);
void vtkROS2ToSlicer(const std_msgs::msg::Float64MultiArray & input, vtkSmartPointer<vtkDoubleArray> result);

// for matrices
void vtkROS2ToSlicer(const std_msgs::msg::Int64MultiArray & input, vtkSmartPointer<vtkTable> result);
void vtkROS2ToSlicer(const std_msgs::msg::Float64MultiArray & input, vtkSmartPointer<vtkTable> result);

// geometry_msgs
void vtkROS2ToSlicer(const geometry_msgs::msg::Pose & input, vtkSmartPointer<vtkMatrix4x4> result);
void vtkROS2ToSlicer(const geometry_msgs::msg::Transform & input, vtkSmartPointer<vtkMatrix4x4> result);
void vtkROS2ToSlicer(const geometry_msgs::msg::Wrench & input, vtkSmartPointer<vtkDoubleArray> result);

// sensor_msgs
void vtkROS2ToSlicer(const sensor_msgs::msg::Image & input, vtkSmartPointer<vtkTypeUInt8Array> result);
void vtkROS2ToSlicer(const sensor_msgs::msg::PointCloud & input, vtkSmartPointer<vtkPoints> result);
void vtkROS2ToSlicer(const sensor_msgs::msg::PointCloud2 & input, vtkSmartPointer<vtkPoints> result);

void vtkROS2ToSlicer(const std_srvs::srv::Trigger::Response & input, vtkSmartPointer<vtkTable> result);
void vtkROS2ToSlicer(const std_srvs::srv::SetBool::Response & input, vtkSmartPointer<vtkBoolString> result);

#endif
