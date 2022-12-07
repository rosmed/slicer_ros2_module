#ifndef __vtkMRMLROS2NodeInternals_h
#define __vtkMRMLROS2NodeInternals_h

#include <rclcpp/rclcpp.hpp>

class vtkMRMLROS2NodeInternals
{
 public:
  std::shared_ptr<rclcpp::Node> mNodePointer;
};

#endif // __vtkMRMLROS2NodeInternals_h
