#include <vtkROS2ToSlicer.h>
#include <vtkMath.h>
#include <vtkVariant.h>
#include <vtkStringArray.h>
#include <vtkFloatArray.h>
#include <vtkIntArray.h>
#include <vtkMRMLROS2Utils.h>
#include <iostream>
using vtkROS2ToSlicerLimits::kMaxPoints;

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
    vtkGenericWarningMacro(<< "Input is not a 1D array");
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
    vtkGenericWarningMacro(<< "Input is not a 1D array");
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
    vtkGenericWarningMacro(<< "Input is not a 2D array");
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
    vtkGenericWarningMacro(<< "Input is not a 2D array");
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
  result->SetElement(0, 3, input.position.x);
  result->SetElement(1, 3, input.position.y);
  result->SetElement(2, 3, input.position.z);
  vtkMRMLROS2::FromSI(result.GetPointer());
}


void vtkROS2ToSlicer(const geometry_msgs::msg::Transform & input, vtkSmartPointer<vtkMatrix4x4> result)
{
  // Basically the same as the function above except the getting method is different
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
  result->SetElement(0, 3, input.translation.x);
  result->SetElement(1, 3, input.translation.y);
  result->SetElement(2, 3, input.translation.z);
  vtkMRMLROS2::FromSI(result.GetPointer());
}


void vtkROS2ToSlicer(const geometry_msgs::msg::Twist & input, vtkSmartPointer<vtkDoubleArray> result)
{
  result->SetNumberOfValues(6);
  result->SetValue(0, input.linear.x);
  result->SetValue(1, input.linear.y);
  result->SetValue(2, input.linear.z);
  result->SetValue(3, input.angular.x);
  result->SetValue(4, input.angular.y);
  result->SetValue(5, input.angular.z);
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

void vtkROS2ToSlicer(const sensor_msgs::msg::Image & input, vtkSmartPointer<vtkImageData> result)
{
  if (!result) {
    return;
  }

  int numberOfComponents = 1;
  int vtkDataType = VTK_UNSIGNED_CHAR;

  if (input.encoding == "rgb8" || input.encoding == "bgr8") {
    numberOfComponents = 3;
  } else if (input.encoding == "rgba8" || input.encoding == "bgra8") {
    numberOfComponents = 4;
  } else if (input.encoding == "mono8") {
    numberOfComponents = 1;
  } else if (input.encoding == "mono16" || input.encoding == "16UC1") {
    numberOfComponents = 1;
    vtkDataType = VTK_UNSIGNED_SHORT;
  } else if (input.encoding == "32FC1") {
    numberOfComponents = 1;
    vtkDataType = VTK_FLOAT;
  }

  result->SetDimensions(input.width, input.height, 1);
  result->AllocateScalars(vtkDataType, numberOfComponents);

  size_t expectedSize = input.width * input.height * numberOfComponents * (vtkDataType == VTK_UNSIGNED_SHORT ? 2 : (vtkDataType == VTK_FLOAT ? 4 : 1));
  if (input.data.size() < expectedSize) {
    vtkGenericWarningMacro(<< "vtkROS2ToSlicer(Image): input data size mismatch. Expected " << expectedSize << " got " << input.data.size());
    return;
  }

  void* pVtk = result->GetScalarPointer();
  std::memcpy(pVtk, input.data.data(), expectedSize);

  // Note: ROS images are top-down, VTK are bottom-up by default, but we'll leave it to the user/logic
  // to apply a flip if needed, or we could do it here if we want to be opinionated.
  result->Modified();
}


void vtkROS2ToSlicer(const sensor_msgs::msg::PointCloud & input, vtkSmartPointer<vtkPoints> result)
{
    if (!result) {
        vtkGenericWarningMacro(<< "vtkROS2ToSlicer(PointCloud): null result");
        return;
    }

    const size_t n = input.points.size();
    if (n > kMaxPoints) {
        vtkGenericWarningMacro(<< "vtkROS2ToSlicer(PointCloud): refusing " << n << " points (max "
                  << kMaxPoints << ")");
        return;
    }

    try {
        result->SetNumberOfPoints(static_cast<vtkIdType>(n));
        for (vtkIdType i = 0; i < static_cast<vtkIdType>(n); ++i) {
            const auto& p = input.points[i];
            result->SetPoint(i, p.x, p.y, p.z);
        }
    } catch (const std::bad_alloc &) {
        vtkGenericWarningMacro(<< "vtkROS2ToSlicer(PointCloud): allocation failed for " << n << " points");
        result->Reset();
    }
}

void vtkROS2ToSlicer(const sensor_msgs::msg::PointCloud2 & input, vtkSmartPointer<vtkPoints> result)
{
    if (!result) {
        vtkGenericWarningMacro(<< "vtkROS2ToSlicer(PointCloud2): null result");
        return;
    }

    result->Reset();

    if (input.data.empty() || input.width == 0 || input.height == 0) {
        vtkGenericWarningMacro(<< "vtkROS2ToSlicer(PointCloud2): empty input");
        return;
    }

    const size_t num_points = static_cast<size_t>(input.width) * input.height;
    if (num_points > kMaxPoints) {
        vtkGenericWarningMacro(<< "vtkROS2ToSlicer(PointCloud2): refusing " << num_points << " points (max "
                  << kMaxPoints << ")");
        return;
    }

    try {
        auto vtkArray = vtkSmartPointer<vtkFloatArray>::New();
        vtkArray->SetNumberOfComponents(3);
        vtkArray->Allocate(num_points * 3);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(input, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(input, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(input, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
                float xyz[3] = { *iter_x, *iter_y, *iter_z };
                vtkArray->InsertNextTuple(xyz);
            }
        }

        result->SetData(vtkArray);
    } catch (const std::bad_alloc &) {
        vtkGenericWarningMacro(<< "vtkROS2ToSlicer(PointCloud2): allocation failed for " << num_points << " points");
        result->Reset();
    }
}

void vtkROS2ToSlicer(const sensor_msgs::msg::PointCloud2 & input, vtkSmartPointer<vtkPolyData> result)
{
  if (!result) {
    return;
  }
  auto points = vtkSmartPointer<vtkPoints>::New();
  vtkROS2ToSlicer(input, points);
  result->SetPoints(points);

  vtkIdType numPoints = points->GetNumberOfPoints();
  if (numPoints > 0) {
    auto vertices = vtkSmartPointer<vtkCellArray>::New();
    vertices->AllocateEstimate(numPoints, 1);
    for (vtkIdType i = 0; i < numPoints; ++i) {
      vertices->InsertNextCell(1, &i);
    }
    result->SetVerts(vertices);
  }
  result->Modified();
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
