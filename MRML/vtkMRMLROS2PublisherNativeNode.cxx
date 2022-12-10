#include <vtkSlicerToROS2.h>

#include <vtkMRMLROS2PublisherNativeNode.h>
#include <vtkMRMLROS2PublisherNativeInternals.h>

VTK_MRML_ROS_PUBLISHER_NATIVE_CXX(std::string, std_msgs::msg::String, String);
VTK_MRML_ROS_PUBLISHER_NATIVE_CXX(bool, std_msgs::msg::Bool, Bool);
