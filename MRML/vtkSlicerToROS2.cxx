#include <vtkSlicerToROS2.h>
#include <vtkMath.h>

const double M_TO_MM = 0.001;

void vtkSlicerToROS2(const std::string & input,  std_msgs::msg::String & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  result.data = input;
}

void vtkSlicerToROS2(const bool & input,  std_msgs::msg::Bool & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  result.data = input;
}

// Work in Progress
void vtkSlicerToROS2(vtkMatrix4x4 * input,  geometry_msgs::msg::PoseStamped & result,
		     const std::shared_ptr<rclcpp::Node> & rosNode)
{
  result.header.frame_id = "slicer"; // VTK 9.2 will support input->GetObjectName();
  result.header.stamp = rosNode->get_clock()->now();
  
  double q[4] = {0.0, 0.0, 0.0, 0.0};
  vtkMatrix4x4ToQuaternion(input, q);
  result.pose.position.x = input->GetElement(0, 3) * M_TO_MM;
  result.pose.position.y = input->GetElement(1, 3) * M_TO_MM;
  result.pose.position.z = input->GetElement(2, 3) * M_TO_MM;
  result.pose.orientation.w = q[0];
  result.pose.orientation.x = q[1];
  result.pose.orientation.y = q[2];
  result.pose.orientation.z = q[3];
}


void vtkSlicerToROS2(vtkMatrix4x4 * input,  geometry_msgs::msg::TransformStamped & result,
		     const std::shared_ptr<rclcpp::Node> & rosNode)
{
  result.header.frame_id = "slicer"; // VTK 9.2 will support input->GetObjectName();
  result.header.stamp = rosNode->get_clock()->now();
  
  double q[4] = {0.0, 0.0, 0.0, 0.0}; 
  vtkMatrix4x4ToQuaternion(input, q);
  result.transform.translation.x = input->GetElement(0, 3) * M_TO_MM;
  result.transform.translation.y = input->GetElement(1, 3) * M_TO_MM;
  result.transform.translation.z = input->GetElement(2, 3) * M_TO_MM;
  result.transform.rotation.w = q[0];
  result.transform.rotation.x = q[1];
  result.transform.rotation.y = q[2];
  result.transform.rotation.z = q[3];
}

void vtkSlicerToROS2(vtkDoubleArray * input, geometry_msgs::msg::WrenchStamped & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
   if (input->GetNumberOfValues() != 6){
     result.wrench.force.x = input->GetValue(0); // for now I'm going to make it 0
     result.wrench.force.y = input->GetValue(1);
     result.wrench.force.z = input->GetValue(2);
     result.wrench.torque.x = input->GetValue(3);
     result.wrench.torque.y = input->GetValue(4);
     result.wrench.torque.z = input->GetValue(5);
   }
   else{
     geometry_msgs::msg::WrenchStamped nullWrench;
     nullWrench.wrench.force.x = 0.0; 
     nullWrench.wrench.force.y = 0.0;
     nullWrench.wrench.force.z = 0.0;
     nullWrench.wrench.torque.x = 0.0;
     nullWrench.wrench.torque.y = 0.0;
     nullWrench.wrench.torque.z = 0.0;
   }
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