#ifndef __vtkMRMLROS2Tf2BufferLookupInternals_h
#define __vtkMRMLROS2Tf2BufferLookupInternals_h

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

class vtkMRMLROS2Tf2BufferLookupInternals
{

 public:

  virtual ~vtkMRMLROS2Tf2BufferLookupInternals() = default;
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
    rosNodePtr->SetNthNodeReferenceID("tf2bufferLookup",
				      rosNodePtr->GetNumberOfNodeReferences("tf2buffer:ookup"),
				      mMRMLNode->GetID());
    mMRMLNode->SetNodeReferenceID("node", nodeId);
    return true;
  }
};

#endif // __vtkMRMLROS2Tf2BufferLookupInternals_h