#ifndef __vtkMRMLROS2SubscriberDefaultNodes_h
#define __vtkMRMLROS2SubscriberDefaultNodes_h

#include <vtkMRMLROS2SubscriberNode.h>
#include <vtkMRMLROS2SubscriberMacros.h>

VTK_MRML_ROS_SUBSCRIBER_NATIVE_H(std::string, String);
VTK_MRML_ROS_SUBSCRIBER_NATIVE_H(bool, Bool);
VTK_MRML_ROS_SUBSCRIBER_NATIVE_H(int, Int);
VTK_MRML_ROS_SUBSCRIBER_NATIVE_H(double, Float);

#include <vtkMatrix4x4.h>
#include <vtkTable.h>
#include <vtkDenseArray.h>

VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkIntArray, IntArray);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkDoubleArray, FloatArray);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkDenseArray<int>, IntNArray);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkTable, Joy);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkMatrix4x4, PoseStamped);

#endif // __vtkMRMLROS2SubscriberDefaultNodes_h
