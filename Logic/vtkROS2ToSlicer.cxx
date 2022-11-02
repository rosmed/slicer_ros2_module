#include <vtkROS2ToSlicer.h>

void vtkROS2ToSlicer(const std_msgs::msg::String & input, std::string & result)
{
// do conversion here
}

void vtkROS2ToSlicer(const geometry_msgs::msg::PoseStamped & input, vtkMatrix4x4 & result)
{
// do conversion here
  std::cerr << "converting " << std::endl;
}
