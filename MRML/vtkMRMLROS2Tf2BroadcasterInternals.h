#ifndef __vtkMRMLROS2Tf2BroadcasterInternals_h
#define __vtkMRMLROS2Tf2BroadcasterInternals_h

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

// SlicerROS2 includes
#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2NodeInternals.h>

// Slicer includes
#include <vtkMRMLScene.h>
#include <vtkMRMLTransformNode.h>
#include <vtkNew.h>
#include <vtkMatrix4x4.h>

auto const MM_TO_M_CONVERSION = 1000.00;

class vtkMRMLROS2Tf2BroadcasterInternals
{
 protected:
  vtkMRMLROS2Tf2BroadcasterNode * mMRMLNode;

 public:
//   vtkMRMLROS2Tf2BroadcasterInternals(vtkMRMLROS2Tf2BroadcasterNode * mrmlNode):
//     mMRMLNode(mrmlNode)
//   {}
  ~vtkMRMLROS2Tf2BroadcasterInternals() = default;
  std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
  std::shared_ptr<rclcpp::Node> mNodePointer;

  bool AddToROS2Node(vtkMRMLScene * scene, const char * nodeId, std::string & errorMessage)
  {
    vtkMRMLNode * rosNodeBasePtr = scene->GetNodeByID(nodeId);
    if (!rosNodeBasePtr) {
      errorMessage = "unable to locate node";
      return false;
    }
    vtkMRMLROS2NODENode * rosNodePtr = dynamic_cast<vtkMRMLROS2NODENode *>(rosNodeBasePtr);
    if (!rosNodePtr) {
      errorMessage = std::string(rosNodeBasePtr->GetName()) + " doesn't seem to be a vtkMRMLROS2NODENode";
      return false;
    }

    mNodePointer = rosNodePtr->mInternals->mNodePointer;
    mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*mNodePointer);
    // This isn't working for some reason
    // rosNodePtr->SetNthNodeReferenceID("tf2broadcaster",
	// 			      rosNodePtr->GetNumberOfNodeReferences("tf2broadcaster"),
	// 			      mMRMLNode->GetID());
    // mMRMLNode->SetNodeReferenceID("node", nodeId);
    return true;
  }

  size_t Broadcast(vtkMRMLTransformNode * message, const std::string & parent_id, const std::string & child_id)
  { 

    geometry_msgs::msg::TransformStamped transformStamped;
    rclcpp::Time now = mNodePointer->get_clock()->now();
    transformStamped.header.stamp = now;
    transformStamped.header.frame_id = parent_id; //"torso"; // should have header be torso and child be base
    transformStamped.child_frame_id = child_id;

    float pos[3] = {0.0, 0.0, 0.0}; // translation vector
    double q[4] = {0.0, 0.0, 0.0, 0.0}; // quaternion vector
    double A[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}}; // 3x3 rotation matrix

    vtkNew<vtkMatrix4x4> matrix;
    message->GetMatrixTransformToParent(matrix);
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 3; ++j) {
        A[i][j] = matrix->GetElement(i, j); // Not sure how to clean up this loop because it's only copying the rotation part of the homeogeneous matrix
      }
    }
    //
    vtkMath::Matrix3x3ToQuaternion(A, q); // Convert quaternion to a 3x3 matrix
    pos[0] = matrix->GetElement(0,3); // Get the translation vector from the homogeneous transformation matrix
    pos[1] = matrix->GetElement(1,3);
    pos[2] = matrix->GetElement(2,3);
    transformStamped.transform.translation.x = pos[0]/MM_TO_M_CONVERSION;
    transformStamped.transform.translation.y = pos[1]/MM_TO_M_CONVERSION;
    transformStamped.transform.translation.z = pos[2]/MM_TO_M_CONVERSION;
    transformStamped.transform.rotation.w = q[0];
    transformStamped.transform.rotation.x = q[1];
    transformStamped.transform.rotation.y = q[2];
    transformStamped.transform.rotation.z = q[3];

    // Send the transform
    mTfBroadcaster->sendTransform(transformStamped);
  }
};

#endif // __vtkMRMLROS2Tf2BroadcasterInternals_h