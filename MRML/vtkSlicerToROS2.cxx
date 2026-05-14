#include <vtkSlicerToROS2.h>
#include <vtkMath.h>

#include <vtkMRMLROS2Utils.h>
using vtkSlicerToROS2Limits::kMaxPoints;

void vtkSlicerToROS2(const std::string &, std_msgs::msg::Empty &,
		     const std::shared_ptr<rclcpp::Node> &)
{
}

void vtkSlicerToROS2(const std::string & input, std_msgs::msg::String & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  result.data = input;
}


void vtkSlicerToROS2(const bool & input, std_msgs::msg::Bool & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  result.data = input;
}


void vtkSlicerToROS2(const int & input, std_msgs::msg::Int64 & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  result.data = input;
}


void vtkSlicerToROS2(const double & input, std_msgs::msg::Float64 & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  result.data = input;
}


void vtkSlicerToROS2(vtkIntArray * input, std_msgs::msg::Int64MultiArray & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  const size_t numElements = input->GetNumberOfValues();
  result.layout.dim.resize(1);
  result.layout.dim[0].label = "x";
  result.layout.dim[0].size = numElements;
  result.layout.dim[0].stride = 1;
  result.data.resize(numElements);
  for (size_t j = 0; j < numElements; j++) {
    result.data[j] = input->GetValue(j);
  }
}


void vtkSlicerToROS2(vtkDoubleArray * input, std_msgs::msg::Float64MultiArray & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  const size_t numElements = input->GetNumberOfValues();
  // set result dim to be 1
  result.layout.dim.resize(1);
  result.layout.dim[0].label = "x";
  result.layout.dim[0].size = numElements;
  result.layout.dim[0].stride = 1;

  result.data.resize(numElements);
  for (size_t j = 0; j < numElements; j++) {
    result.data[j] = input->GetValue(j);
  }
}


void vtkSlicerToROS2(vtkTable * input, std_msgs::msg::Int64MultiArray & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  const size_t numCols = input->GetNumberOfColumns();
  const size_t numRows = input->GetNumberOfRows();

  result.layout.dim.resize(2);
  result.layout.dim[0].label = "x";
  result.layout.dim[0].size = numRows;
  result.layout.dim[0].stride = numCols;
  result.layout.dim[1].label = "y";
  result.layout.dim[1].size = numCols;
  result.layout.dim[1].stride = 1;

  result.data.resize(numRows*numCols);

  for (size_t i = 0; i < numRows; i++) {
    for (size_t j = 0; j < numCols; j++) {
      result.data[i*numCols + j] = input->GetValue(i, j).ToInt();
    }
  }
}


void vtkSlicerToROS2(vtkTable * input, std_msgs::msg::Float64MultiArray & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  const size_t numCols = input->GetNumberOfColumns();
  const size_t numRows = input->GetNumberOfRows();

  result.layout.dim.resize(2);
  result.layout.dim[0].label = "x";
  result.layout.dim[0].size = numRows;
  result.layout.dim[0].stride = numCols;
  result.layout.dim[1].label = "y";
  result.layout.dim[1].size = numCols;
  result.layout.dim[1].stride = 1;

  result.data.resize(numRows * numCols);

  for (size_t i = 0; i < numRows; i++) {
    for (size_t j = 0; j < numCols; j++) {
      result.data[i*numCols + j] = input->GetValue(i, j).ToDouble();
    }
  }
}


void vtkSlicerToROS2(vtkMatrix4x4 * input, geometry_msgs::msg::Pose & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  double q[4] = {0.0, 0.0, 0.0, 0.0};
  vtkMatrix4x4ToQuaternion(input, q);
  result.position.x = vtkMRMLROS2::ToSI(input->GetElement(0, 3));
  result.position.y = vtkMRMLROS2::ToSI(input->GetElement(1, 3));
  result.position.z = vtkMRMLROS2::ToSI(input->GetElement(2, 3));
  result.orientation.w = q[0];
  result.orientation.x = q[1];
  result.orientation.y = q[2];
  result.orientation.z = q[3];
}


void vtkSlicerToROS2(vtkMatrix4x4 * input, geometry_msgs::msg::Transform & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  double q[4] = {0.0, 0.0, 0.0, 0.0};
  vtkMatrix4x4ToQuaternion(input, q);
  result.translation.x = vtkMRMLROS2::ToSI(input->GetElement(0, 3));
  result.translation.y = vtkMRMLROS2::ToSI(input->GetElement(1, 3));
  result.translation.z = vtkMRMLROS2::ToSI(input->GetElement(2, 3));
  result.rotation.w = q[0];
  result.rotation.x = q[1];
  result.rotation.y = q[2];
  result.rotation.z = q[3];
}


void vtkSlicerToROS2(vtkDoubleArray * input, geometry_msgs::msg::Twist & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  if (input->GetNumberOfValues() == 6) {
    result.linear.x = input->GetValue(0);
    result.linear.y = input->GetValue(1);
    result.linear.z = input->GetValue(2);
    result.angular.x = input->GetValue(3);
    result.angular.y = input->GetValue(4);
    result.angular.z = input->GetValue(5);
  } else {
    result.linear.x = 0.0;
    result.linear.y = 0.0;
    result.linear.z = 0.0;
    result.angular.x = 0.0;
    result.angular.y = 0.0;
    result.angular.z = 0.0;
  }
}


void vtkSlicerToROS2(vtkDoubleArray * input, geometry_msgs::msg::Wrench & result,
		     const std::shared_ptr<rclcpp::Node> &)
{
  if (input->GetNumberOfValues() == 6) {
    result.force.x = input->GetValue(0);
    result.force.y = input->GetValue(1);
    result.force.z = input->GetValue(2);
    result.torque.x = input->GetValue(3);
    result.torque.y = input->GetValue(4);
    result.torque.z = input->GetValue(5);
  } else {
    result.force.x = 0.0;
    result.force.y = 0.0;
    result.force.z = 0.0;
    result.torque.x = 0.0;
    result.torque.y = 0.0;
    result.torque.z = 0.0;
  }
}


void vtkSlicerToROS2(vtkTransformCollection * input, geometry_msgs::msg::PoseArray & result,
		     const std::shared_ptr<rclcpp::Node> & rosNode)
{
  result.header.frame_id = "slicer"; // VTK 9.2 will support input->GetObjectName();
  result.header.stamp = rosNode->get_clock()->now();
  result.poses.clear();

  const size_t numItems = input->GetNumberOfItems();
  for (size_t i = 0; i < numItems; ++i) {
    vtkTransform * transform = vtkTransform::SafeDownCast(input->GetItemAsObject(i));
    if (transform) {
      vtkMatrix4x4 * matrix = transform->GetMatrix();
      geometry_msgs::msg::Pose pose;

      double q[4] = {0.0, 0.0, 0.0, 0.0};
      vtkMatrix4x4ToQuaternion(matrix, q);
      pose.position.x = vtkMRMLROS2::ToSI(matrix->GetElement(0, 3));
      pose.position.y = vtkMRMLROS2::ToSI(matrix->GetElement(1, 3));
      pose.position.z = vtkMRMLROS2::ToSI(matrix->GetElement(2, 3));
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
  const size_t numberOfValues = input->GetNumberOfValues();
  for (size_t i = 0; i < numberOfValues; ++i) {
    picture.push_back(input->GetValue(i));
  }
  result.data = picture;
}

void vtkSlicerToROS2(vtkImageData * input, sensor_msgs::msg::Image & result,
                     const std::shared_ptr<rclcpp::Node> & rosNode)
{
  if (!input) {
    return;
  }
  result.header.stamp = rosNode->get_clock()->now();
  int dims[3];
  input->GetDimensions(dims);
  result.width = dims[0];
  result.height = dims[1];
  
  int numberOfComponents = input->GetNumberOfScalarComponents();
  int vtkDataType = input->GetScalarType();

  if (vtkDataType == VTK_UNSIGNED_CHAR) {
    if (numberOfComponents == 1) result.encoding = "mono8";
    else if (numberOfComponents == 3) result.encoding = "rgb8";
    else if (numberOfComponents == 4) result.encoding = "rgba8";
  } else if (vtkDataType == VTK_UNSIGNED_SHORT && numberOfComponents == 1) {
    result.encoding = "16UC1";
  } else if (vtkDataType == VTK_FLOAT && numberOfComponents == 1) {
    result.encoding = "32FC1";
  } else {
    std::cerr << "vtkSlicerToROS2(Image): unsupported VTK data type or component count" << std::endl;
    return;
  }

  size_t pixelSize = input->GetScalarSize() * numberOfComponents;
  size_t dataSize = result.width * result.height * pixelSize;
  result.step = result.width * pixelSize;
  result.data.resize(dataSize);
  std::memcpy(result.data.data(), input->GetScalarPointer(), dataSize);
}


void vtkSlicerToROS2(vtkPoints * input, sensor_msgs::msg::PointCloud & result, 
                    const std::shared_ptr<rclcpp::Node> & rosNode)
{
    result.header.stamp = rosNode->get_clock()->now();
    if (!input) {
        std::cerr << "vtkSlicerToROS2(PointCloud): null input" << std::endl;
        return;
    }
    
    const vtkIdType n = input->GetNumberOfPoints();
    if (n < 0 || static_cast<size_t>(n) > kMaxPoints) {
        std::cerr << "vtkSlicerToROS2(PointCloud): refusing " << n << " points (max "
                  << kMaxPoints << ")" << std::endl;
        return;
    }
    
    try {
        result.points.clear();
        result.points.reserve(n);
        
        for (vtkIdType i = 0; i < n; ++i) {
            double p[3];
            input->GetPoint(i, p);
            
            geometry_msgs::msg::Point32 pt;
            pt.x = static_cast<float>(p[0]);
            pt.y = static_cast<float>(p[1]);
            pt.z = static_cast<float>(p[2]);
            result.points.push_back(pt);
        }
    } catch (const std::bad_alloc &) {
        std::cerr << "vtkSlicerToROS2(PointCloud): allocation failed for " << n << " points" << std::endl;
        result.points.clear();
    }
}

void vtkSlicerToROS2(vtkPoints * input, sensor_msgs::msg::PointCloud2 & result, 
                      const std::shared_ptr<rclcpp::Node> & rosNode)
{
    result.header.stamp = rosNode->get_clock()->now();
    if (!input) {
        std::cerr << "vtkSlicerToROS2(PointCloud2): null input" << std::endl;
        return;
    }
    
    const vtkIdType n = input->GetNumberOfPoints();
    if (n < 0 || static_cast<size_t>(n) > kMaxPoints) {
        std::cerr << "vtkSlicerToROS2(PointCloud2): refusing " << n << " points (max "
                  << kMaxPoints << ")" << std::endl;
        return;
    }
    
    try {
        result.height = 1;
        result.width = static_cast<uint32_t>(n);
        result.is_bigendian = false;
        result.is_dense = true;
        
        sensor_msgs::PointCloud2Modifier modifier(result);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(n);
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(result, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(result, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(result, "z");
        
        for (vtkIdType i = 0; i < n; ++i, ++iter_x, ++iter_y, ++iter_z) {
            double p[3];
            input->GetPoint(i, p);
            *iter_x = static_cast<float>(p[0]);
            *iter_y = static_cast<float>(p[1]);
            *iter_z = static_cast<float>(p[2]);
        }
    } catch (const std::bad_alloc &) {
        std::cerr << "vtkSlicerToROS2(PointCloud2): allocation failed for " << n << " points" << std::endl;
        result = sensor_msgs::msg::PointCloud2{};
    }
}

void vtkSlicerToROS2(vtkPolyData * input, sensor_msgs::msg::PointCloud2 & result, 
                      const std::shared_ptr<rclcpp::Node> & rosNode)
{
  if (!input) {
    return;
  }
  vtkSlicerToROS2(input->GetPoints(), result, rosNode);
}


void vtkSlicerToROS2(vtkBool * input, std_srvs::srv::SetBool::Request & result,
                     const std::shared_ptr<rclcpp::Node> &)
{
  result.data = input->GetValue();
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
