#ifndef __vtkROS2ToSlicer_h
#define __vtkROS2ToSlicer_h

#include "vtkSlicerRos2ModuleLogicExport.h"
#include <vtkMRMLROS2SubscriberNode.h>
//
inline void vtkROS2ToSlicer(const std_msgs::msg::String & input, vtkStdString & result);

inline void vtkROS2ToSlicer(const geometry_msgs::msg::PoseStamped & input, vtkMatrix4x4 & result);

#endif
