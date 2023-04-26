#ifndef __vtkMRMLROS2CISST_h
#define __vtkMRMLROS2CISST_h

#include <SlicerROS2Config.h>

#if USE_CISST_MSGS

#include <vtkMatrix4x4.h>
#include <vtkMRMLROS2PublisherNode.h>
#include <vtkMRMLROS2PublisherMacros.h>

VTK_MRML_ROS_PUBLISHER_VTK_H(vtkMatrix4x4, CartesianImpedanceGains);

#endif // USE_CISST_MSGS

#endif // _SlicerROS2Config_h
