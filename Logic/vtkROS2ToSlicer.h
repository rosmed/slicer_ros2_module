#ifndef __vtkROS2ToSlicer_h
#define __vtkROS2ToSlicer_h

// VTK
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

// ROS2
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


#include "vtkSlicerRos2ModuleLogicExport.h"

void vtkROS2ToSlicer(const std_msgs::msg::String & input, vtkSmartPointer<std::string> result);
void vtkROS2ToSlicer(const geometry_msgs::msg::PoseStamped & input, vtkSmartPointer<vtkMatrix4x4> result);

#endif
