#include <vtkROS2ToSlicer.h>

#include <vtkMRMLROS2SubscriberDefaultNodes.h>
#include <vtkMRMLROS2SubscriberInternals.h>

VTK_MRML_ROS_SUBSCRIBER_NATIVE_CXX(std_msgs::msg::String, std::string, String);
VTK_MRML_ROS_SUBSCRIBER_NATIVE_CXX(std_msgs::msg::Bool, bool, Bool);

VTK_MRML_ROS_SUBSCRIBER_VTK_CXX(sensor_msgs::msg::Joy, vtkTable, Joy)
VTK_MRML_ROS_SUBSCRIBER_VTK_CXX(geometry_msgs::msg::PoseStamped, vtkMatrix4x4, PoseStamped)
