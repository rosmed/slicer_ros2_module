#ifndef __vtkMRMLROS2PublisherDefaultsNodes_h
#define __vtkMRMLROS2PublisherDefaultsNodes_h

#include <vtkMRMLROS2PublisherNode.h>
#include <vtkMRMLROS2PublisherMacros.h>

VTK_MRML_ROS_PUBLISHER_NATIVE_H(std::string, String);
VTK_MRML_ROS_PUBLISHER_NATIVE_H(bool, Bool);

#include <vtkMatrix4x4.h>
#include <vtkTransformCollection.h>

VTK_MRML_ROS_PUBLISHER_VTK_H(vtkMatrix4x4, PoseStamped);
VTK_MRML_ROS_PUBLISHER_VTK_H(vtkDoubleArray, WrenchStamped);
VTK_MRML_ROS_PUBLISHER_VTK_H(vtkTransformCollection, PoseArray);
VTK_MRML_ROS_PUBLISHER_VTK_H(vtkMatrix4x4, CartesianImpedanceGains);

#endif // __vtkMRMLROS2PublisherDefaultsNodes_h
