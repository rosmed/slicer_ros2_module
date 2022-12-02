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
void vtkSlicerToROS2(const vtkSmartPointer<vtkMatrix4x4> & input,  geometry_msgs::msg::PoseStamped result)
{
  result.pose.position.x = input->GetElement(0,3)*M_TO_MM_CONVERSION;
  result.pose.position.y = input->GetElement(1,3)*M_TO_MM_CONVERSION;
  result.pose.position.z = input->GetElement(2,3)*M_TO_MM_CONVERSION;
}

// void vtkSlicerToROS2(const geometry_msgs::msg::PoseStamped & input, vtkSmartPointer<vtkMatrix4x4> result)
// {
//   // Get individual elements from the ros message
//   auto x = input.pose.position.x*MM_TO_M_CONVERSION;
//   auto y = input.pose.position.y*MM_TO_M_CONVERSION;
//   auto z = input.pose.position.z*MM_TO_M_CONVERSION;
//   auto q_w = input.pose.orientation.w;
//   auto q_x = input.pose.orientation.x;
//   auto q_y = input.pose.orientation.y;
//   auto q_z = input.pose.orientation.z;

// //   // Copy contents into a vtkMRMLTransformNode
//   const double q[4] = {q_w, q_x, q_y, q_z};
//   double A[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};



// //   // Is there a more efficient way to do this??
// //   // Apply rotation
//   vtkMath::QuaternionToMatrix3x3(q, A); // Convert quaternion to a 3x3 matrix?
//   for (size_t row = 0; row < 3; row++) {
//     for (size_t column = 0; column < 3; column++) {
//       A[row][column] = input->GetElement(row, column, A[row][column]); // Set the 3x3 matrix as the rotation component of the homogeneous transform
//      }
//   }

//   // Apply translation vector
//   result->SetElement(0, 3, x);
//   result->SetElement(1, 3, y);
//   result->SetElement(2, 3, z);
// }
