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

void vtkSlicerToROS2(const int & input,  std_msgs::msg::Int64 & result,
         const std::shared_ptr<rclcpp::Node> &)
{
  result.data = input;
}

void vtkSlicerToROS2(const double & input,  std_msgs::msg::Float64 & result,
         const std::shared_ptr<rclcpp::Node> &)
{
  result.data = input;
}

void vtkSlicerToROS2(vtkIntArray * input,  std_msgs::msg::Int64MultiArray & result,
         const std::shared_ptr<rclcpp::Node> &)
{
  int numElements = input->GetNumberOfValues();
  result.data.resize(numElements);
  for (int j = 0; j < numElements; j++){
    result.data[j] = input->GetValue(j);
  }
}

void vtkSlicerToROS2(vtkDoubleArray * input,  std_msgs::msg::Float64MultiArray & result,
         const std::shared_ptr<rclcpp::Node> &)
{
  int numElements = input->GetNumberOfValues();
  result.data.resize(numElements);
  for (int j = 0; j < numElements; j++){
    result.data[j] = input->GetValue(j);
  }
}

void vtkSlicerToROS2(vtkDenseArray<int> * input,  std_msgs::msg::Int64MultiArray & result,
         const std::shared_ptr<rclcpp::Node> &) 
{
  // vtkArrayExtents extents = input->GetExtents();
  // int x = extents.GetSize()[0];
  // int y = extents.GetSize()[1];

  // result.data.resize(x*y);
  // for (int i = 0; i < x; i++){
  //   for (int j = 0; j < y; j++){
  //     result.data[i*y + j] = input->GetValue(i,j);
  //   }
  // }
  // result.layout.dim.resize(2);
  // result.layout.dim[0].label = "x";
  // result.layout.dim[0].size = x;
  // result.layout.dim[0].stride = x*y;
  // result.layout.dim[1].label = "y";
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
   if (input->GetNumberOfValues() == 6){
     result.wrench.force.x = input->GetValue(0); // for now I'm going to make it 0
     result.wrench.force.y = input->GetValue(1);
     result.wrench.force.z = input->GetValue(2);
     result.wrench.torque.x = input->GetValue(3);
     result.wrench.torque.y = input->GetValue(4);
     result.wrench.torque.z = input->GetValue(5);
   }
   else{
     result.wrench.force.x = 0.0; 
     result.wrench.force.y = 0.0;
     result.wrench.force.z = 0.0;
     result.wrench.torque.x = 0.0;
     result.wrench.torque.y = 0.0;
     result.wrench.torque.z = 0.0;
   }
}

void vtkSlicerToROS2(vtkTransformCollection * input, geometry_msgs::msg::PoseArray & result,
		     const std::shared_ptr<rclcpp::Node> & rosNode)
{
  result.header.frame_id = "slicer"; // VTK 9.2 will support input->GetObjectName();
  result.header.stamp = rosNode->get_clock()->now();
  result.poses.clear();

  for (int i = 0; i < input->GetNumberOfItems(); i++){
    vtkTransform* transform = vtkTransform::SafeDownCast(input->GetItemAsObject(i));
    if (transform){
      vtkMatrix4x4* matrix = transform->GetMatrix();
      geometry_msgs::msg::Pose pose;

      double q[4] = {0.0, 0.0, 0.0, 0.0};
      vtkMatrix4x4ToQuaternion(matrix, q);
      pose.position.x = matrix->GetElement(0, 3) * M_TO_MM;
      pose.position.y = matrix->GetElement(1, 3) * M_TO_MM;
      pose.position.z = matrix->GetElement(2, 3) * M_TO_MM;
      pose.orientation.w = q[0];
      pose.orientation.x = q[1];
      pose.orientation.y = q[2];
      pose.orientation.z = q[3];

      result.poses.push_back(pose);
    }
  }
}

void vtkSlicerToROS2(vtkMatrix4x4 * input, cisst_msgs::msg::CartesianImpedanceGains & result,
		     const std::shared_ptr<rclcpp::Node> &) // the input should be something related to the closest point on the volume 
{
  // stiffness = elasticity
  // damping = viscosity
  result.pos_stiff_neg.x = 0.0;
  result.pos_stiff_pos.x = 0.0;
  result.pos_damping_neg.x = 0.0;
  result.pos_damping_pos.x = 0.0;
  result.pos_stiff_neg.y = 0.0;
  result.pos_stiff_pos.y = 0.0;
  result.pos_damping_neg.y = 0.0;
  result.pos_damping_pos.y = 0.0;
  // These gains are preconfigured for our application - they can be modified according to your device/ specific needs
  result.pos_stiff_neg.z = -10.0;
  result.pos_stiff_pos.z = 10.0; 
  result.pos_damping_neg.z = -20.0; 
  result.pos_damping_pos.z = 20.0; 

  double stiffOri = -0.2;
  double dampOri = -0.01;
  result.ori_stiff_neg.x = stiffOri;
  result.ori_stiff_pos.x = stiffOri;
  result.ori_damping_neg.x = dampOri;
  result.ori_damping_pos.x = dampOri;
  result.ori_stiff_neg.y = stiffOri;
  result.ori_stiff_pos.y = stiffOri;
  result.ori_damping_neg.y = dampOri;
  result.ori_damping_pos.y = dampOri;
  result.ori_stiff_neg.z = 0.0;
  result.ori_stiff_pos.z = 0.0;
  result.ori_damping_neg.z = 0.0;
  result.ori_damping_pos.z = 0.0;

  result.force_position.x = input->GetElement(0, 3) * M_TO_MM;
  result.force_position.y = input->GetElement(1, 3) * M_TO_MM;
  result.force_position.z = input->GetElement(2, 3) * M_TO_MM;

  double q[4] = {0.0, 0.0, 0.0, 0.0};
  vtkMatrix4x4ToQuaternion(input, q);
  result.force_orientation.x = q[1];
  result.force_orientation.y = q[2];
  result.force_orientation.z = q[3]; // should this come from breach warning 
  result.force_orientation.w = q[0]; 
  result.torque_orientation.x = q[1];
  result.torque_orientation.y = q[2];
  result.torque_orientation.z = q[3];
  result.torque_orientation.w = q[0];
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