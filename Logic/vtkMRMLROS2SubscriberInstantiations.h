#ifndef __vtkMRMLROS2SubscriberInstantiations_h
#define __vtkMRMLROS2SubscriberInstantiations_h

#include <vtkROS2ToSlicer.h>
#include <vtkMRMLROS2SubscriberImplementation.h>

typedef vtkMRMLROS2SubscriberImplementation<std_msgs::msg::String, std::string> vtkMRMLROS2SubscriberString;

typedef vtkMRMLROS2SubscriberImplementation<geometry_msgs::msg::PoseStamped, vtkSmartPointer<vtkMatrix4x4>> vtkMRMLROS2SubscriberPoseStamped;

#endif
