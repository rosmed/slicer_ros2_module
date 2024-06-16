#include <vtkROS2ToSlicer.h>
#include <vtkMath.h>
#include <vtkVariant.h>


auto const MM_TO_M_CONVERSION = 1000.00;

void vtkROS2ToSlicer(const std_msgs::msg::String & input, std::string & result)
{
  result = input.data;
}

void vtkROS2ToSlicer(const std_msgs::msg::Bool & input, bool & result)
{
  result = input.data;
}

void vtkROS2ToSlicer(const std_msgs::msg::Int64 & input, int & result)
{
  result = input.data;
}

void vtkROS2ToSlicer(const std_msgs::msg::Float64 & input, double & result)
{
  result = input.data;
}

void vtkROS2ToSlicer(const std_msgs::msg::Int64MultiArray & input, vtkSmartPointer<vtkIntArray> result)
{
  int numElements = input.data.size();
  // if input is not a 1D array raise an error
  if (input.layout.dim.size() != 1){
    std::cerr << "Input is not a 1D array" << std::endl;
    return;
  }
  result->SetNumberOfValues(numElements);
  for (int j = 0; j < numElements; j++){
    result->SetValue(j, input.data[j]);
  }
}

void vtkROS2ToSlicer(const std_msgs::msg::Float64MultiArray & input, vtkSmartPointer<vtkDoubleArray> result)
{
  int numElements = input.data.size();
  // if input is not a 1D array raise an error
  if (input.layout.dim.size() != 1){
    std::cerr << "Input is not a 1D array" << std::endl;
    return;
  }
  result->SetNumberOfValues(numElements);
  for (int j = 0; j < numElements; j++){
    result->SetValue(j, input.data[j]);
  }
}

void vtkROS2ToSlicer(const std_msgs::msg::Int64MultiArray & input, vtkSmartPointer<vtkTable> result)
{
  // if input is not a 2D array raise an error
  if (input.layout.dim.size() != 2){
    std::cerr << "Input is not a 2D array" << std::endl;
    return;
  }
  int numRows = input.layout.dim[0].size;
  int numCols = input.layout.dim[1].size;
  for(int i = 0; i < numCols; i++){
    vtkSmartPointer<vtkIntArray> col = vtkSmartPointer<vtkIntArray>::New();
    col->SetNumberOfValues(numRows);
    for(int j = 0; j < numRows; j++){
      col->SetValue(j, input.data[j*numCols + i]);
    }
    result->AddColumn(col);
  }
}

void vtkROS2ToSlicer(const std_msgs::msg::Float64MultiArray & input, vtkSmartPointer<vtkTable> result)
{
  // if input is not a 2D array raise an error
  if (input.layout.dim.size() != 2){
    std::cerr << "Input is not a 2D array" << std::endl;
    return;
  }
  int numRows = input.layout.dim[0].size;
  int numCols = input.layout.dim[1].size;
  for(int i = 0; i < numCols; i++){
    vtkSmartPointer<vtkDoubleArray> col = vtkSmartPointer<vtkDoubleArray>::New();
    col->SetNumberOfValues(numRows);
    for(int j = 0; j < numRows; j++){
      col->SetValue(j, input.data[j*numCols + i]);
    }
    result->AddColumn(col);
  }
}

void vtkROS2ToSlicer(const sensor_msgs::msg::Joy & input, vtkSmartPointer<vtkTable> result)
{
  result->SetNumberOfRows(2); // Row 1 = button status, Row 2 = axes values
  int numAxes = input.axes.size();
  int numButtons = input.buttons.size();
  for (int j = 0; j < numAxes; j++){
    result->SetValue(1, j, vtkVariant(input.axes[j]));
  }
  for (int j = 0; j < numButtons; j++){
    result->SetValue(0, j, vtkVariant(input.buttons[j]));
  }
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

  // Is there a more efficient way to do this??
  // Apply rotation
  vtkMath::QuaternionToMatrix3x3(q, A); // Convert quaternion to a 3x3 matrix
  for (size_t row = 0; row < 3; row++) {
    for (size_t column = 0; column < 3; column++) {
      result->SetElement(row, column, A[row][column]); // Set the 3x3 matrix as the rotation component of the homogeneous transform
    }
  }

  // Apply translation vector
  result->SetElement(0, 3, x);
  result->SetElement(1, 3, y);
  result->SetElement(2, 3, z);
}

void vtkROS2ToSlicer(const geometry_msgs::msg::TransformStamped & input, vtkSmartPointer<vtkMatrix4x4> result)
{
  // Basically the same as the function above except the getting method is different
  // Get individual elements from the ros message
  auto x = input.transform.translation.x*MM_TO_M_CONVERSION;
  auto y = input.transform.translation.y*MM_TO_M_CONVERSION;
  auto z = input.transform.translation.z*MM_TO_M_CONVERSION;
  auto q_w = input.transform.rotation.w;
  auto q_x = input.transform.rotation.x;
  auto q_y = input.transform.rotation.y;
  auto q_z = input.transform.rotation.z;

  // Copy contents into a vtkMRMLTransformNode
  const double q[4] = {q_w, q_x, q_y, q_z};
  double A[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};

  // Is there a more efficient way to do this??
  // Apply rotation
  vtkMath::QuaternionToMatrix3x3(q, A); // Convert quaternion to a 3x3 matrix
  for (size_t row = 0; row < 3; row++) {
    for (size_t column = 0; column < 3; column++) {
      result->SetElement(row, column, A[row][column]); // Set the 3x3 matrix as the rotation component of the homogeneous transform
    }
  }

  // Apply translation vector
  result->SetElement(0, 3, x);
  result->SetElement(1, 3, y);
  result->SetElement(2, 3, z);
}

void vtkROS2ToSlicer(const sensor_msgs::msg::Image& input, vtkSmartPointer<vtkTypeUInt8Array> result)
{
    // Initialize the array to the correct size
    result->SetNumberOfComponents(input.width);
    result->SetNumberOfTuples(input.height);

    // Populate the array with the data from the ros image
    for (std::vector<unsigned char>::size_type i = 0; i < input.data.size(); i++) {
      result->InsertValue(i, input.data[i]);
    }
}

void vtkROS2ToSlicer(const sensor_msgs::msg::PointCloud & input, vtkSmartPointer<vtkPoints> result)
{
    // Initialize the vtkPoints
    result->Reset();
    result->Allocate(input.points.size());

    // Iterate through all points in the input PointCloud
    for (const auto& point : input.points) {
        // Add each point to the vtkPoints object
        result->InsertNextPoint(point.x, point.y, point.z);
    }
}