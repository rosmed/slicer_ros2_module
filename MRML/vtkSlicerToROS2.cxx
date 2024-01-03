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
  result.layout.dim.resize(1);
  result.layout.dim[0].label = "x";
  result.layout.dim[0].size = numElements;
  result.layout.dim[0].stride = 1;
  result.data.resize(numElements);
  for (int j = 0; j < numElements; j++){
    result.data[j] = input->GetValue(j);
  }
}

void vtkSlicerToROS2(vtkDoubleArray * input,  std_msgs::msg::Float64MultiArray & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  int numElements = input->GetNumberOfValues();
  // set result dim to be 1
  result.layout.dim.resize(1);
  result.layout.dim[0].label = "x";
  result.layout.dim[0].size = numElements;
  result.layout.dim[0].stride = 1;

  result.data.resize(numElements);
  for (int j = 0; j < numElements; j++){
    result.data[j] = input->GetValue(j);
  }
}

void vtkSlicerToROS2(vtkTable * input,  std_msgs::msg::Int64MultiArray & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  int numCols = input->GetNumberOfColumns();
  int numRows = input->GetNumberOfRows();

  result.layout.dim.resize(2);
  result.layout.dim[0].label = "x";
  result.layout.dim[0].size = numRows;
  result.layout.dim[0].stride = numCols;
  result.layout.dim[1].label = "y";
  result.layout.dim[1].size = numCols;
  result.layout.dim[1].stride = 1;

  result.data.resize(numRows*numCols);

  for (int i = 0; i < numRows; i++){
    for (int j = 0; j < numCols; j++){
      result.data[i*numCols + j] = input->GetValue(i, j).ToInt();
    }
  }
}

void vtkSlicerToROS2(vtkTable * input,  std_msgs::msg::Float64MultiArray & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  int numCols = input->GetNumberOfColumns();
  int numRows = input->GetNumberOfRows();

  result.layout.dim.resize(2);
  result.layout.dim[0].label = "x";
  result.layout.dim[0].size = numRows;
  result.layout.dim[0].stride = numCols;
  result.layout.dim[1].label = "y";
  result.layout.dim[1].size = numCols;
  result.layout.dim[1].stride = 1;

  result.data.resize(numRows*numCols);

  for (int i = 0; i < numRows; i++){
    for (int j = 0; j < numCols; j++){
      result.data[i*numCols + j] = input->GetValue(i, j).ToDouble();
    }
  }
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

void vtkSlicerToROS2(vtkTypeUInt8Array * input, sensor_msgs::msg::Image & result,
         const std::shared_ptr<rclcpp::Node> & rosNode)
{
  result.header.stamp = rosNode->get_clock()->now();
  std::vector<uint8_t> picture; 
  result.width = input->GetNumberOfComponents();
  result.height = input->GetNumberOfTuples();
  result.encoding = "mono8"; // grayscale for ultrasound
  int numberOfValues = input->GetNumberOfValues();
  for (int i = 0; i < numberOfValues; i ++){
    picture.push_back(input->GetValue(i));
  } 
  result.data = picture;
}


void vtkSlicerToROS2(vtkTable * input, std_srvs::srv::SetBool::Request & result, const std::shared_ptr<rclcpp::Node> & rosNode)
{
    if (!input || !input->GetNumberOfRows() || !input->GetColumn(0))
    {
        std::cerr << "Invalid input table" << std::endl;
        return;
    }

    vtkIntArray* boolArray = vtkIntArray::SafeDownCast(input->GetColumn(0));
    if (!boolArray)
    {
        std::cerr << "First column is not an int array" << std::endl;
        return;
    }

    // Assuming the boolean value is stored in the first row of the first column
    int boolValue = boolArray->GetValue(0);
    result.data = (boolValue != 0); // Convert int to bool

    //     # Create a new vtkTable
    // table = vtk.vtkTable()

    // # Create an vtkIntArray for storing boolean values
    // boolArray = vtk.vtkIntArray()
    // boolArray.SetName("BooleanValue")

    // # Add the array to the table
    // table.AddColumn(boolArray)

    // # Set the number of rows in the table
    // table.SetNumberOfRows(1)

    // # Insert a false value (0 in this case) into the first row
    // table.SetValue(0, 0, 0) 
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
