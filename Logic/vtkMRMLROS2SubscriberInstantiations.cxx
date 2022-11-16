#include <vtkMRMLROS2SubscriberInstantiations.h>

template<>
vtkStandardNewMacro(vtkMRMLROS2SubscriberString);
// vtkMRMLNodeNewMacro()
template<>
vtkMRMLNode* vtkMRMLROS2SubscriberImplementation<std_msgs::msg::String, std::string>::CreateNodeInstance() 
{
  return SelfType::New(); 
}

template<>
vtkStandardNewMacro(vtkMRMLROS2SubscriberPoseStamped);
template<>
vtkMRMLNode* vtkMRMLROS2SubscriberImplementation<geometry_msgs::msg::PoseStamped, vtkSmartPointer<vtkMatrix4x4> >::CreateNodeInstance() 
{
  return SelfType::New(); 
}