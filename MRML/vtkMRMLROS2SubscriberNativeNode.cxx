#include <vtkROS2ToSlicer.h>

#include <vtkMRMLROS2SubscriberNativeNode.h>

VTK_MRML_ROS_SUBSCRIBER_NATIVE_CXX(std_msgs::msg::String, std::string, String);
VTK_MRML_ROS_SUBSCRIBER_NATIVE_CXX(std_msgs::msg::Bool, bool, Bool);
