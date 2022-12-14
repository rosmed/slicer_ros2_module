#include <vtkROS2ToSlicer.h>

#include <vtkMRMLROS2ParameterNativeNode.h>
#include <vtkMRMLROS2ParameterNativeInternals.h>

VTK_MRML_ROS_PARAMETER_NATIVE_CXX(std_msgs::msg::String, std::string, String);
VTK_MRML_ROS_PARAMETER_NATIVE_CXX(std_msgs::msg::Bool, bool, Bool);
