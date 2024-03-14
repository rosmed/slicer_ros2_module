#include <vtkSlicerToROS2.h>
#include <vtkROS2ToSlicer.h>
#include <vtkMRMLROS2ServiceClientDefaultNodes.h>
#include <vtkMRMLROS2ServiceClientInternals.h>

// VTK_MRML_ROS_SERVICE_CLIENT_NATIVE_CXX(std::string, std_msgs::msg::String, String);
// VTK_MRML_ROS_SERVICE_CLIENT_NATIVE_CXX(bool, std_msgs::msg::Bool, Bool);
// VTK_MRML_ROS_SERVICE_CLIENT_NATIVE_CXX(int, std_msgs::msg::Int64, Int);
// VTK_MRML_ROS_SERVICE_CLIENT_NATIVE_CXX(double, std_msgs::msg::Float64, Double);

// VTK_MRML_ROS_SERVICE_CLIENT_VTK_CXX(vtkIntArray, std_msgs::msg::Int64MultiArray, IntArray);
// VTK_MRML_ROS_SERVICE_CLIENT_VTK_CXX(vtkDoubleArray, std_msgs::msg::Float64MultiArray, DoubleArray);
VTK_MRML_ROS_SERVICE_CLIENT_VTK_CXX(vtkTable, vtkTable, std_srvs::srv::SetBool, SetBool); 
VTK_MRML_ROS_SERVICE_CLIENT_VTK_CXX(vtkBoolString, vtkBoolString, std_srvs::srv::SetBool, SetBoolString); 
// VTK_MRML_ROS_SERVICE_CLIENT_VTK_CXX(vtkTable, std_msgs::msg::Float64MultiArray, DoubleTable);

// VTK_MRML_ROS_SERVICE_CLIENT_VTK_CXX(vtkMatrix4x4, geometry_msgs::msg::PoseStamped, PoseStamped);
// VTK_MRML_ROS_SERVICE_CLIENT_VTK_CXX(vtkDoubleArray, geometry_msgs::msg::WrenchStamped, WrenchStamped);
// VTK_MRML_ROS_SERVICE_CLIENT_VTK_CXX(vtkTransformCollection, geometry_msgs::msg::PoseArray, PoseArray);
// VTK_MRML_ROS_SERVICE_CLIENT_VTK_CXX(vtkTypeUInt8Array, sensor_msgs::msg::Image, UInt8Image);
