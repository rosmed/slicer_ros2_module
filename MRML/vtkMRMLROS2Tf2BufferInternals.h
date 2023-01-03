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

class vtkMRMLROS2Tf2BufferInternals
{

 public:

  virtual ~vtkMRMLROS2Tf2BufferInternals() = default;
  std::shared_ptr<tf2_ros::Buffer> mTfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> mTfListener;
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
    // Should we have a GetTf2BufferNodeByName (probably child and parent id's)
    mTfBuffer = std::make_unique<tf2_ros::Buffer>(mNodePointer->get_clock());
    mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);
    rosNodePtr->SetNthNodeReferenceID("tf2buffer",
				      rosNodePtr->GetNumberOfNodeReferences("tf2buffer"),
				      mMRMLNode->GetID());
    mMRMLNode->SetNodeReferenceID("node", nodeId);
    return true;
  }

  bool AddLookupAndCreateNode(vtkMRMLScene * scene, const std::string & parent_id, const std::string & child_id, std::string & errorMessage)
  {
    try {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped = mTfBuffer->lookupTransform(parent_id, child_id, tf2::TimePointZero);  
      vtkNew<vtkMatrix4x4> matrix;
      vtkROS2ToSlicer(transformStamped, matrix);
      vtkSmartPointer<vtkMRMLLinearTransformNode> transform = vtkSmartPointer<vtkMRMLLinearTransformNode>::New();
      transform->SetMatrixTransformToParent(matrix);
      transform->SetName((parent_id + "To" + child_id).c_str());
      scene->AddNode(transform);
      return true;
    } 
    catch (tf2::TransformException & ex) {
      errorMessage = "Could not find the transform between " + parent_id + " and " +  child_id; 
      return false;
    }
  }

  bool AddLookupForExistingNode(vtkMRMLScene * scene, const std::string & parent_id, const std::string & child_id, const std::string transformID, std::string & errorMessage )
  {
    vtkMRMLTransformNode *transform = vtkMRMLTransformNode::SafeDownCast(scene->GetNodeByID(transformID));
    if (!transform){
        errorMessage = "Transform does not exist for provided ID.";
        return false;
    }
    try {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped = mTfBuffer->lookupTransform(parent_id, child_id, tf2::TimePointZero);  
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
  }
};

#endif // __vtkMRMLROS2Tf2BufferInternals_h