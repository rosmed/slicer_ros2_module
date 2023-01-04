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
#include <vtkSlicerToROS2.h>
#include <vtkMRMLTransformNode.h> // should this go here 
#include <vtkMatrix4x4.h>

class vtkMRMLROS2Tf2BroadcasterInternals
{

 public:

  virtual ~vtkMRMLROS2Tf2BroadcasterInternals() = default;
  std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
  std::shared_ptr<rclcpp::Node> mNodePointer;

  bool AddToROS2Node(vtkMRMLNode * mMRMLNode, vtkMRMLScene * scene, const char * nodeId, std::string & errorMessage)
  {
    vtkMRMLNode * rosNodeBasePtr = scene->GetNodeByID(nodeId);
    if (!rosNodeBasePtr) {
      errorMessage = "Unable to locate ros2 node in the scene";
      return false;
    }
    vtkMRMLROS2NODENode * rosNodePtr = dynamic_cast<vtkMRMLROS2NODENode *>(rosNodeBasePtr);
    if (!rosNodePtr) {
      errorMessage = std::string(rosNodeBasePtr->GetName()) + " doesn't seem to be a vtkMRMLROS2NODENode";
      return false;
    }
    
    mNodePointer = rosNodePtr->mInternals->mNodePointer;
    // Should we have a GetTf2BroadcasterNodeByName (probably child and parent id's)
    mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*mNodePointer);
    rosNodePtr->SetNthNodeReferenceID("tf2broadcaster",
				      rosNodePtr->GetNumberOfNodeReferences("tf2broadcaster"),
				      mMRMLNode->GetID());
    mMRMLNode->SetNodeReferenceID("node", nodeId);
    return true;
  }

  
  size_t Broadcast(vtkMRMLTransformNode * message, const std::string & parent_id, const std::string & child_id)
  { 
    geometry_msgs::msg::TransformStamped rosTransform;
    vtkNew<vtkMatrix4x4> matrix;
    message->GetMatrixTransformToParent(matrix);
    vtkSlicerToROS2(matrix, rosTransform);
    rclcpp::Time now = mNodePointer->get_clock()->now();
    rosTransform.header.stamp = now;
    rosTransform.header.frame_id = parent_id;
    rosTransform.child_frame_id = child_id;

    // Send the transform
    mTfBroadcaster->sendTransform(rosTransform);
    return true;
  }

  size_t Broadcast(vtkMatrix4x4 * message, const std::string & parent_id, const std::string & child_id)
  { 
    geometry_msgs::msg::TransformStamped rosTransform;
    vtkSlicerToROS2(message, rosTransform);
    rclcpp::Time now = mNodePointer->get_clock()->now(); // SlicerToROS2 could do this
    rosTransform.header.stamp = now;
    rosTransform.header.frame_id = parent_id;
    rosTransform.child_frame_id = child_id;

    // Send the transform
    mTfBroadcaster->sendTransform(rosTransform);
    return true;
  }
};

#endif // __vtkMRMLROS2Tf2BroadcasterInternals_h
