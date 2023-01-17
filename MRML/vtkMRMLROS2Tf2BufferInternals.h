#ifndef __vtkMRMLROS2Tf2BufferInternals_h
#define __vtkMRMLROS2Tf2BufferInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// SlicerROS2 includes
#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2NodeInternals.h>

// Slicer includes
#include <vtkMRMLScene.h>
#include <vtkROS2ToSlicer.h>
#include <vtkMRMLLinearTransformNode.h> // should this go here 
#include <vtkMatrix4x4.h>
#include <vtkMRMLROS2Tf2BufferNode.h>
#include <vtkMRMLROS2Tf2LookupNode.h>

class vtkMRMLROS2Tf2BufferInternals
{

 public:

  virtual ~vtkMRMLROS2Tf2BufferInternals() = default;
  std::shared_ptr<tf2_ros::Buffer> mTfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> mTfListener;
  std::shared_ptr<rclcpp::Node> mNodePointer;

  bool lookupTryCatch(const std::string & parent_id, const std::string & child_id, std::string & errorMessage, vtkMRMLROS2Tf2LookupNode * lookup, vtkMRMLTransformNode * transform)
  { 
    try {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped = mTfBuffer->lookupTransform(parent_id, child_id, tf2::TimePointZero);  // check how old we want the data to be (right now it's doing it no matter how old) - for now we don't care
      vtkNew<vtkMatrix4x4> matrix;
      vtkROS2ToSlicer(transformStamped, matrix);
      transform->SetMatrixTransformToParent(matrix);
      transform->Modified();
      return true;
    } 
    catch (tf2::TransformException & ex) {
      errorMessage = "Could not find the transform between " + parent_id + " and " + child_id; 
      return false;
    }
    catch(...){
      errorMessage = "Got an undefined exception.";
      return false;
    }
    return false;
  }
};

#endif // __vtkMRMLROS2Tf2BufferInternals_h