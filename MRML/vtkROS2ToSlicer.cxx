#include <vtkROS2ToSlicer.h>
#include <vtkMath.h>
#include <vtkVariant.h>
#include <vtkStringArray.h>
#include <vtkIntArray.h>
#include <vtkFloatArray.h>

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

void vtkROS2ToSlicer(const sensor_msgs::msg::PointCloud2 & input, vtkSmartPointer<vtkPoints> result)
{
    // A PointCloud2 is a data structure for storing arrays of points such that the points can have multiple 
    // attributes like x,y,z,intensity or R,A,S,label. The PointCloud2 is structured like a table with a width and 
    // height - this is how we can determine the size of the point cloud. In each of the elements of the table, or points
    // in the cloud, we have a field that tells you the name of the attribute (ie. "x"), the byte offset of that field in the 
    // point for accessing, the type of data (eg. FLOAT), and the number of elements in the field (usually one).
    // You can aslo determine whether or not the PointCloud is big or little endian, the number of bytes per element and
    // per row, whether or not the point cloud contains naan points or not and then finally the data as an array.

    // Initialize the vtkPoints
    result->Reset();
    size_t num_points = input.width * input.height;
    result->Allocate(num_points);

    // Check if there is data in the PointCloud2
    if (input.data.empty() || input.width == 0 || input.height == 0) {
        std::cerr << "No points in the PointCloud2" << std::endl;
        return;
    }

    // Get the field offsets for x, y, z
    int offset_x = -1, offset_y = -1, offset_z = -1;
    for (const auto& field : input.fields) {
        if (field.name == "x") offset_x = field.offset;
        if (field.name == "y") offset_y = field.offset;
        if (field.name == "z") offset_z = field.offset;
    }

    // Ensure x, y, and z offset fields exist
    if (offset_x == -1 || offset_y == -1 || offset_z == -1) {
        std::cerr << "Offset data does not exist." << std::endl;
        return;
    }

    const uint8_t *data = input.data.data();
    size_t point_step = input.point_step; // Size of each point in bytes
    vtkSmartPointer<vtkFloatArray> vtkArray = vtkSmartPointer<vtkFloatArray>::New();
    vtkArray->SetNumberOfComponents(3); // x, y, z
    vtkArray->SetNumberOfTuples(num_points);
    for (size_t i = 0; i < num_points; ++i) {
        const uint8_t *point_data = data + i * point_step;
        float x, y, z;
        memcpy(&x, point_data + offset_x, sizeof(float));
        memcpy(&y, point_data + offset_y, sizeof(float));
        memcpy(&z, point_data + offset_z, sizeof(float));

        // Check if the point is valid
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            vtkArray->SetTuple3(i, x, y, z);
        }
    }

    // Set the vtkPoints with the data in the vtkArray
    result->SetData(vtkArray);
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

