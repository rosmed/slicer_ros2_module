#include <vtkROS2ToSlicer.h>


inline void vtkROS2ToSlicer(const std_msgs::msg::String & input, vtkStdString & result)
{
// do conversion here
}

inline void vtkROS2ToSlicer(const geometry_msgs::msg::PoseStamped & input, vtkMatrix4x4 & result)
{
// do conversion here
  std::cerr << "converting " << std::endl;
}
