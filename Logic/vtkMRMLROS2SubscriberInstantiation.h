#ifndef __vtkMRMLROS2SubscriberInstantiation_h
#define __vtkMRMLROS2SubscriberInstantiation_h

#include <vtkMRMLROS2SubscriberNode.h>
#include <vtkMRMLROS2SubscriberImplementation.h>

typedef vtkMRMLROS2SubscriberImplementation<geometry_msgs::msg::PoseStamped, vtkMatrix4x4> vtkMRMLROS2SubscriberPoseStamped;
typedef vtkMRMLROS2SubscriberImplementation<std_msgs::msg::String, vtkStdString> vtkMRMLROS2SubscriberString;

#endif