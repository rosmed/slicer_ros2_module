#ifndef __vtkMRMLROS2SubscriberDefaultNodes_h
#define __vtkMRMLROS2SubscriberDefaultNodes_h

#include <vtkMRMLROS2SubscriberNode.h>
#include <vtkMRMLROS2SubscriberMacros.h>

VTK_MRML_ROS_SUBSCRIBER_NATIVE_H(std::string, String);
VTK_MRML_ROS_SUBSCRIBER_NATIVE_H(bool, Bool);
VTK_MRML_ROS_SUBSCRIBER_NATIVE_H(int, Int);
VTK_MRML_ROS_SUBSCRIBER_NATIVE_H(double, Double);

#include <vtkMatrix4x4.h>
#include <vtkTable.h>
#include <vtkTable.h>
#include <vtkTypeUInt8Array.h>
#include <vtkPoints.h>

VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkIntArray, IntArray);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkDoubleArray, DoubleArray);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkTable, IntTable);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkTable, DoubleTable);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkTable, Joy);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkMatrix4x4, PoseStamped);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkTypeUInt8Array, UInt8Image);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkPoints, PointCloud);

#endif // __vtkMRMLROS2SubscriberDefaultNodes_h
