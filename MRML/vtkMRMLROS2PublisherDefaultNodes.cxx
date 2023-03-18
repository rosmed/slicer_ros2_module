#include <vtkSlicerToROS2.h>

#include <vtkMRMLROS2PublisherDefaultNodes.h>
#include <vtkMRMLROS2PublisherInternals.h>

VTK_MRML_ROS_PUBLISHER_NATIVE_CXX(std::string, std_msgs::msg::String, String);
VTK_MRML_ROS_PUBLISHER_NATIVE_CXX(bool, std_msgs::msg::Bool, Bool);

VTK_MRML_ROS_PUBLISHER_VTK_CXX(vtkMatrix4x4, geometry_msgs::msg::PoseStamped, PoseStamped);
VTK_MRML_ROS_PUBLISHER_VTK_CXX(vtkDoubleArray, geometry_msgs::msg::WrenchStamped, WrenchStamped);
VTK_MRML_ROS_PUBLISHER_VTK_CXX(vtkTransformCollection, geometry_msgs::msg::PoseArray, PoseArray);
