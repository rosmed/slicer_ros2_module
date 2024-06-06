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

#include <vtkROS2GeometryMsgsPoseStamped.h>
#include <vtkROS2SensorMsgsJoy.h>
#include <vtkROS2SensorMsgsJointState.h>
#include <vtkROS2GeometryMsgsWrenchStamped.h>

VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkIntArray, IntArray);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkDoubleArray, DoubleArray);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkTable, IntTable);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkTable, DoubleTable);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkTable, Joy);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkGeometryMsgsPoseStamped, PoseStamped);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkMatrix4x4, Pose);

VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkDoubleArray, Wrench);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkGeometryMsgsWrenchStamped, WrenchStamped);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkSensorMsgsJoy, JoyV2);
VTK_MRML_ROS_SUBSCRIBER_VTK_H(vtkSensorMsgsJointState, JointState);

#endif // __vtkMRMLROS2SubscriberDefaultNodes_h
