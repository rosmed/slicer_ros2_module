#ifndef __vtkSlicerToROS2_h
#define __vtkSlicerToROS2_h

// VTK
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

// ROS2
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


#include "vtkSlicerRos2ModuleLogicExport.h"

void vtkSlicerToROS2(const std::string & input,  std_msgs::msg::String & result);
void vtkSlicerToROS2(const bool & input,  std_msgs::msg::Bool & result);
void vtkSlicerToROS2(const vtkSmartPointer<vtkMatrix4x4> & input,  geometry_msgs::msg::PoseStamped result);

#endif
