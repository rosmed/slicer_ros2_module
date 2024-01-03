#ifndef __vtkMRMLROS2ServiceClientDefaultsNodes_h
#define __vtkMRMLROS2ServiceClientDefaultsNodes_h

#include <vtkMRMLROS2ServiceClientNode.h>
#include <vtkMRMLROS2ServiceClientMacros.h>

// VTK_MRML_ROS_SERVICE_CLIENT_NATIVE_H(std::string, String);
// VTK_MRML_ROS_SERVICE_CLIENT_NATIVE_H(bool, Bool);
// VTK_MRML_ROS_SERVICE_CLIENT_NATIVE_H(int, Int);
// VTK_MRML_ROS_SERVICE_CLIENT_NATIVE_H(double, Double);

// #include <vtkMatrix4x4.h>
// #include <vtkTransformCollection.h>
#include <vtkTable.h>
// #include <vtkTypeUInt8Array.h>

// VTK_MRML_ROS_SERVICE_CLIENT_VTK_H(vtkIntArray, IntArray);
// VTK_MRML_ROS_SERVICE_CLIENT_VTK_H(vtkDoubleArray, DoubleArray);
VTK_MRML_ROS_SERVICE_CLIENT_VTK_H(vtkTable, vtkTable, SetBool); // Input Output Name
// VTK_MRML_ROS_SERVICE_CLIENT_VTK_H(vtkTable, DoubleTable);
// VTK_MRML_ROS_SERVICE_CLIENT_VTK_H(vtkMatrix4x4, PoseStamped);
// VTK_MRML_ROS_SERVICE_CLIENT_VTK_H(vtkDoubleArray, WrenchStamped);
// VTK_MRML_ROS_SERVICE_CLIENT_VTK_H(vtkTransformCollection, PoseArray);
// VTK_MRML_ROS_SERVICE_CLIENT_VTK_H(vtkTypeUInt8Array, UInt8Image);

#endif // __vtkMRMLROS2ServiceClientDefaultsNodes_h
