#include <vtkSlicerToROS2.h>
#include <vtkMath.h>

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
  double q[4] = {0.0, 0.0, 0.0, 0.0};
  vtkMatrix4x4ToQuaternion(input, q);

  result.pose.position.x = input->GetElement(0,3)*M_TO_MM_CONVERSION;
  result.pose.position.y = input->GetElement(1,3)*M_TO_MM_CONVERSION;
  result.pose.position.z = input->GetElement(2,3)*M_TO_MM_CONVERSION;
  result.header.frame_id = "slicer";
  result.pose.orientation.w = q[0];
  result.pose.orientation.x = q[1];
  result.pose.orientation.y = q[2];
  result.pose.orientation.z = q[3];
}


void vtkSlicerToROS2(vtkMatrix4x4 * matrix,  geometry_msgs::msg::TransformStamped & result)
{
  double q[4] = {0.0, 0.0, 0.0, 0.0}; 
  vtkMatrix4x4ToQuaternion(matrix, q);
  
  result.transform.translation.x = matrix->GetElement(0,3)*M_TO_MM_CONVERSION;
  result.transform.translation.y = matrix->GetElement(1,3)*M_TO_MM_CONVERSION;
  result.transform.translation.z = matrix->GetElement(2,3)*M_TO_MM_CONVERSION;
  result.transform.rotation.w = q[0];
  result.transform.rotation.x = q[1];
  result.transform.rotation.y = q[2];
  result.transform.rotation.z = q[3];
}

void vtkMatrix4x4ToQuaternion(vtkMatrix4x4 * input, double quaternion[4])
{
  double A[3][3];
  for (size_t row = 0; row < 3; row++) {
    for (size_t column = 0; column < 3; column++) {
      A[row][column] = input->GetElement(row, column); // Get the 3x3 matrix rotation component of the homogeneous transform
     }
  }
  vtkMath::Matrix3x3ToQuaternion(A, quaternion);
}