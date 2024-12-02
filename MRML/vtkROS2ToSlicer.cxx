#include <vtkROS2ToSlicer.h>
#include <vtkMath.h>
#include <vtkVariant.h>
#include <vtkStringArray.h>
#include <vtkIntArray.h>

auto const MM_TO_M_CONVERSION = 1000.00;

void vtkROS2ToSlicer(const std_msgs::msg::Empty &, std::string &)
{
}

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
  const size_t numElements = input.data.size();
  // if input is not a 1D array raise an error
  if (input.layout.dim.size() != 1) {
    std::cerr << "Input is not a 1D array" << std::endl;
    return;
  }
  result->SetNumberOfValues(numElements);
  for (size_t i = 0; i < numElements; ++i) {
    result->SetValue(i, input.data[i]);
  }
}


void vtkROS2ToSlicer(const std_msgs::msg::Float64MultiArray & input, vtkSmartPointer<vtkDoubleArray> result)
{
  const size_t numElements = input.data.size();
  // if input is not a 1D array raise an error
  if (input.layout.dim.size() != 1) {
    std::cerr << "Input is not a 1D array" << std::endl;
    return;
  }
  result->SetNumberOfValues(numElements);
  for (size_t i = 0; i < numElements; ++i) {
    result->SetValue(i, input.data[i]);
  }
}


void vtkROS2ToSlicer(const std_msgs::msg::Int64MultiArray & input, vtkSmartPointer<vtkTable> result)
{
  // if input is not a 2D array raise an error
  if (input.layout.dim.size() != 2) {
    std::cerr << "Input is not a 2D array" << std::endl;
    return;
  }
  const size_t numRows = input.layout.dim[0].size;
  const size_t numCols = input.layout.dim[1].size;
  for (size_t i = 0; i < numCols; ++i) {
    vtkSmartPointer<vtkIntArray> col = vtkSmartPointer<vtkIntArray>::New();
    col->SetNumberOfValues(numRows);
    for (size_t j = 0; j < numRows; ++j) {
      col->SetValue(j, input.data[j*numCols + i]);
    }
    result->AddColumn(col);
  }
}


void vtkROS2ToSlicer(const std_msgs::msg::Float64MultiArray & input, vtkSmartPointer<vtkTable> result)
{
  // if input is not a 2D array raise an error
  if (input.layout.dim.size() != 2) {
    std::cerr << "Input is not a 2D array" << std::endl;
    return;
  }
  const size_t numRows = input.layout.dim[0].size;
  const size_t numCols = input.layout.dim[1].size;
  for (size_t i = 0; i < numCols; ++i) {
    vtkSmartPointer<vtkDoubleArray> col = vtkSmartPointer<vtkDoubleArray>::New();
    col->SetNumberOfValues(numRows);
    for (size_t j = 0; j < numRows; ++j) {
      col->SetValue(j, input.data[j*numCols + i]);
    }
    result->AddColumn(col);
  }
}


void vtkROS2ToSlicer(const geometry_msgs::msg::Pose & input, vtkSmartPointer<vtkMatrix4x4> result)
{
  // Get individual elements from the ros message
  auto x = input.position.x * MM_TO_M_CONVERSION;
  auto y = input.position.y * MM_TO_M_CONVERSION;
  auto z = input.position.z * MM_TO_M_CONVERSION;
  auto q_w = input.orientation.w;
  auto q_x = input.orientation.x;
  auto q_y = input.orientation.y;
  auto q_z = input.orientation.z;

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


void vtkROS2ToSlicer(const geometry_msgs::msg::Transform & input, vtkSmartPointer<vtkMatrix4x4> result)
{
  // Basically the same as the function above except the getting method is different
  // Get individual elements from the ros message
  auto x = input.translation.x * MM_TO_M_CONVERSION;
  auto y = input.translation.y * MM_TO_M_CONVERSION;
  auto z = input.translation.z * MM_TO_M_CONVERSION;
  auto q_w = input.rotation.w;
  auto q_x = input.rotation.x;
  auto q_y = input.rotation.y;
  auto q_z = input.rotation.z;

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


void vtkROS2ToSlicer(const geometry_msgs::msg::Wrench & input, vtkSmartPointer<vtkDoubleArray> result)
{
  result->SetNumberOfValues(6);
  result->SetValue(0, input.force.x);
  result->SetValue(1, input.force.y);
  result->SetValue(2, input.force.z);
  result->SetValue(3, input.torque.x);
  result->SetValue(4, input.torque.y);
  result->SetValue(5, input.torque.z);
}


void vtkROS2ToSlicer(const sensor_msgs::msg::Image & input, vtkSmartPointer<vtkTypeUInt8Array> result)
{
    // Initialize the array to the correct size
    result->SetNumberOfComponents(input.width);
    result->SetNumberOfTuples(input.height);

    // Populate the array with the data from the ros image
    for (std::vector<unsigned char>::size_type i = 0; i < input.data.size(); ++i) {
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


void vtkROS2ToSlicer(const std_srvs::srv::Trigger::Response & input, vtkSmartPointer<vtkTable> result)
{
    vtkSmartPointer<vtkStringArray> messageArray = vtkSmartPointer<vtkStringArray>::New();
    messageArray->SetName("message");
    result->AddColumn(messageArray);

    vtkSmartPointer<vtkIntArray> successArray = vtkSmartPointer<vtkIntArray>::New();
    successArray->SetName("success");
    result->AddColumn(successArray);

    result->SetNumberOfRows(1);
    result->SetValue(0, 0, vtkVariant(input.message));
    result->SetValue(0, 1, vtkVariant(input.success));
}


void vtkROS2ToSlicer(const std_srvs::srv::SetBool::Response & input, vtkSmartPointer<vtkBoolString> result)
{
  result->SetResult(input.success);
  result->SetMessage(input.message);
}
