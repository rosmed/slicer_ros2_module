#include <vtkROS2ToSlicer.h>

#include <vtkMRMLROS2SubscriberVTKNode.h>

VTK_MRML_ROS_SUBSCRIBER_VTK_CXX(geometry_msgs::msg::PoseStamped, vtkMatrix4x4, PoseStamped)
