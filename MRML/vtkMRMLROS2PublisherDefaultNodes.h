#ifndef __vtkMRMLROS2PublisherDefaultsNodes_h
#define __vtkMRMLROS2PublisherDefaultsNodes_h

#include <vtkMRMLROS2PublisherNode.h>
#include <vtkMRMLROS2PublisherMacros.h>

VTK_MRML_ROS_PUBLISHER_NATIVE_H(std::string, String);
VTK_MRML_ROS_PUBLISHER_NATIVE_H(bool, Bool);
VTK_MRML_ROS_PUBLISHER_NATIVE_H(int, Int);
VTK_MRML_ROS_PUBLISHER_NATIVE_H(double, Double);

#include <vtkMatrix4x4.h>
#include <vtkTransformCollection.h>
#include <vtkTable.h>
#include <vtkTypeUInt8Array.h>

VTK_MRML_ROS_PUBLISHER_VTK_H(vtkIntArray, IntArray);
VTK_MRML_ROS_PUBLISHER_VTK_H(vtkDoubleArray, DoubleArray);
VTK_MRML_ROS_PUBLISHER_VTK_H(vtkTable, IntTable);
VTK_MRML_ROS_PUBLISHER_VTK_H(vtkTable, DoubleTable);
VTK_MRML_ROS_PUBLISHER_VTK_H(vtkMatrix4x4, PoseStamped);
VTK_MRML_ROS_PUBLISHER_VTK_H(vtkDoubleArray, WrenchStamped);
VTK_MRML_ROS_PUBLISHER_VTK_H(vtkTransformCollection, PoseArray);
VTK_MRML_ROS_PUBLISHER_VTK_H(vtkTypeUInt8Array, UInt8Image);

#endif // __vtkMRMLROS2PublisherDefaultsNodes_h
