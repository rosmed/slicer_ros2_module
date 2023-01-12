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

  bool AddToROS2Node(vtkMRMLNode * mMRMLNode, vtkMRMLROS2Tf2BufferNode * buff, vtkMRMLScene * scene, const char * nodeId, std::string & errorMessage) // did something weird here with buff - use mMRMLNode instead
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
    vtkMRMLROS2Tf2BufferNode * buffer = rosNodePtr->GetBufferNodeByID(mMRMLNode->GetID()); // check if ptr is set ( don't need to check ID ) - should also have a GetBuffer method
    if (buffer != nullptr){
      errorMessage = "This buffer has already been added to the ROS2 node.";
      return false;
    }
    mTfBuffer = std::make_unique<tf2_ros::Buffer>(mNodePointer->get_clock());
    mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);
    rosNodePtr->SetNthNodeReferenceID("Tf2buffers",
				      rosNodePtr->GetNumberOfNodeReferences("Tf2buffers"),
				      mMRMLNode->GetID());
    mMRMLNode->SetNodeReferenceID("node", nodeId);
    rosNodePtr->mBuffer = buff;
    return true;
  }

  bool InstantiateLookups(vtkMRMLScene * scene, const std::string & parent_id, const std::string & child_id, std::string & errorMessage, vtkMRMLROS2Tf2LookupNode * lookup)
  {

    vtkMRMLTransformNode *transform = vtkMRMLTransformNode::SafeDownCast(scene->GetNodeByID(lookup->GetID()));
    if (!transform){
        errorMessage = "Transform does not exist for provided ID.";
        return false;
    }
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
    // can do a cascaded  catch - other exceptions check documentation
  }
};

#endif // __vtkMRMLROS2Tf2BufferInternals_h