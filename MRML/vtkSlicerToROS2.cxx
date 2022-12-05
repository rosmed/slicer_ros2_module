#include <vtkSlicerToROS2.h>
#include <vtkMath.h>
// #include <rclcpp/time.hpp>

auto const M_TO_MM_CONVERSION = 0.001;

void vtkSlicerToROS2(const std::string & input,  std_msgs::msg::String & result)
{
  result.data = input;
}

void vtkSlicerToROS2(const bool & input,  std_msgs::msg::Bool & result)
{
  result.data = input;
}

// Work in Progress
void vtkSlicerToROS2(vtkMatrix4x4 * input,  geometry_msgs::msg::PoseStamped & result)
{
  result.pose.position.x = input->GetElement(0,3)*M_TO_MM_CONVERSION;
  result.pose.position.y = input->GetElement(1,3)*M_TO_MM_CONVERSION;
  result.pose.position.z = input->GetElement(2,3)*M_TO_MM_CONVERSION;
  std::cerr << result.pose.position.x << result.pose.position.y << result.pose.position.z << std::endl;

  double q[4];
  double A[3][3];

  for (size_t row = 0; row < 3; row++) {
    for (size_t column = 0; column < 3; column++) {
      A[row][column] = input->GetElement(row, column); // Get the 3x3 matrix rotation component of the homogeneous transform
     }
  }

  vtkMath::Matrix3x3ToQuaternion(A,q);
  result.header.frame_id = "blah blah custom generated frame";
  result.pose.orientation.w = q[0];
  result.pose.orientation.x = q[1];
  result.pose.orientation.y = q[2];
  result.pose.orientation.z = q[3];
}

