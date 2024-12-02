#ifndef __vtkSlicerToROS2_h
#define __vtkSlicerToROS2_h

#include <cstdint>

// VTK
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <vtkDoubleArray.h>
#include <vtkIntArray.h>
#include <vtkTransformCollection.h>
#include <vtkTable.h>
#include <vtkTypeUInt8Array.h>
#include <vtkPoints.h>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

// new
#include "std_srvs/srv/set_bool.hpp"

// custom types
#include <vtkCustomTypes.h>

// std_msgs
void vtkSlicerToROS2(const std::string & input, std_msgs::msg::Empty & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);
void vtkSlicerToROS2(const std::string & input, std_msgs::msg::String & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);
void vtkSlicerToROS2(const bool & input, std_msgs::msg::Bool & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);
void vtkSlicerToROS2(const int & input, std_msgs::msg::Int64 & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);
void vtkSlicerToROS2(const double & input, std_msgs::msg::Float64 & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);
// for vectors
void vtkSlicerToROS2(vtkIntArray * input, std_msgs::msg::Int64MultiArray & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);
void vtkSlicerToROS2(vtkDoubleArray * input, std_msgs::msg::Float64MultiArray & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);
// for matrices
void vtkSlicerToROS2(vtkTable * input, std_msgs::msg::Int64MultiArray & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);
void vtkSlicerToROS2(vtkTable * input, std_msgs::msg::Float64MultiArray & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);

// geometry_msgs
void vtkSlicerToROS2(vtkMatrix4x4 * input, geometry_msgs::msg::Pose & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);
void vtkSlicerToROS2(vtkMatrix4x4 * input, geometry_msgs::msg::Transform & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);
void vtkSlicerToROS2(vtkDoubleArray * input, geometry_msgs::msg::Wrench & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);
void vtkSlicerToROS2(vtkTransformCollection * input, geometry_msgs::msg::PoseArray & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);

// sensor_msgs
void vtkSlicerToROS2(vtkTypeUInt8Array * input, sensor_msgs::msg::Image & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);
void vtkSlicerToROS2(vtkPoints * input, sensor_msgs::msg::PointCloud & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);

void vtkSlicerToROS2(vtkBool * input, std_srvs::srv::SetBool::Request & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode);

// helper function
void vtkMatrix4x4ToQuaternion(vtkMatrix4x4 * input, double quaternion[4]);

#endif // __vtkSlicerToROS2_h
