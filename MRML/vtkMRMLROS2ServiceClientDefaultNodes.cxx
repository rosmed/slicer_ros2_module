#include <vtkSlicerToROS2.h>
#include <vtkROS2ToSlicer.h>
#include <vtkMRMLROS2ServiceClientDefaultNodes.h>
#include <vtkMRMLROS2ServiceClientInternals.h>

// VTK_MRML_ROS_SERVICE_CLIENT_VTK_CXX(vtkTable, vtkTable, std_srvs::srv::SetBool, SetBool); 
VTK_MRML_ROS_SERVICE_CLIENT_VTK_CXX(vtkBool, vtkBoolString, std_srvs::srv::SetBool, SetBoolString); 

