#include <vtkROS2ToSlicer.h>

#include <vtkMRMLROS2SubscriberVTKNode.h>
#include <vtkMRMLROS2SubscriberVTKInternals.h>

VTK_MRML_ROS_SUBSCRIBER_VTK_CXX(geometry_msgs::msg::PoseStamped, vtkMatrix4x4, PoseStamped)
