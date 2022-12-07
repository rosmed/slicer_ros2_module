#include <vtkSlicerToROS2.h>

#include <vtkMRMLROS2PublisherVTKNode.h>

VTK_MRML_ROS_PUBLISHER_VTK_CXX(vtkMatrix4x4, geometry_msgs::msg::PoseStamped, PoseStamped);
