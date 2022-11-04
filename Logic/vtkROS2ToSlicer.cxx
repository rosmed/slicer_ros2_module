#include <vtkROS2ToSlicer.h>
#include <vtkMath.h>

auto const MM_TO_M_CONVERSION = 1000.00;

void vtkROS2ToSlicer(const std_msgs::msg::String & input, std::string & result)
{
// do conversion here

}

void vtkROS2ToSlicer(const geometry_msgs::msg::PoseStamped & input, vtkSmartPointer<vtkMatrix4x4> result)
{
  // Get individual elements from the ros message
  auto x = input.pose.position.x*MM_TO_M_CONVERSION;
  auto y = input.pose.position.y*MM_TO_M_CONVERSION;
  auto z = input.pose.position.z*MM_TO_M_CONVERSION;
  auto q_w = input.pose.orientation.w;
  auto q_x = input.pose.orientation.x;
  auto q_y = input.pose.orientation.y;
  auto q_z = input.pose.orientation.z;

  // Copy contents into a vtkMRMLTransformNode
  const double q[4] = {q_w, q_x, q_y, q_z};
  double A[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};

  // Apply rotation
  vtkMath::QuaternionToMatrix3x3(q, A); // Convert quaternion to a 3x3 matrix
//   vtkNew<vtkMatrix4x4> Tf; // Figure out how to capture this properly
  vtkNew<vtkMatrix4x4> Tf;
  for (size_t row = 0; row < 3; row++) {
    for (size_t column = 0; column < 3; column++) {
      Tf->SetElement(row, column, A[row][column]); // Set the 3x3 matrix as the rotation component of the homogeneous transform
     }
  }

  // Apply translation vector
  Tf->SetElement(0,3, x);
  Tf->SetElement(1,3, y);
  Tf->SetElement(2,3, z);
  result = Tf;
  std::cerr << "Test" << result->GetElement(0,0) <<std::endl;
}
