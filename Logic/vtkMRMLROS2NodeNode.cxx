#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2NodeInternals.h>
#include "vtkCommand.h"

vtkStandardNewMacro(vtkMRMLROS2NodeNode);

vtkMRMLNode * vtkMRMLROS2NodeNode::CreateNodeInstance(void)
{
  return SelfType::New();
}

const char * vtkMRMLROS2NodeNode::GetNodeTagName(void)
{
  return "ROS2Node";
}

vtkMRMLROS2NodeNode::vtkMRMLROS2NodeNode()
{
  mInternals = std::make_unique<vtkMRMLROS2NodeInternals>();
}

vtkMRMLROS2NodeNode::~vtkMRMLROS2NodeNode()
{
}

void vtkMRMLROS2NodeNode::Create(const std::string & nodeName, bool initialize)
{
  // - this should be detected automatically by look for node of type ROS2Node in scene
  // - there might also be a rclcpp method to detect if the context has been initialized
  if (initialize) {
    typedef char * char_pointer;
    char_pointer * argv = new char_pointer[1];
    argv[0]= new char[nodeName.size() + 1];
    strcpy(argv[0], nodeName.c_str());
    int argc = 1;
    rclcpp::init(argc, argv);
  }

  // create the ROS node
  mMRMLNodeName = "ros2:node:" + nodeName;
  this->SetName(mMRMLNodeName.c_str());
  mInternals->mNodePointer = std::make_shared<rclcpp::Node>(nodeName);
}

void vtkMRMLROS2NodeNode::Spin(void)
{
  if (rclcpp::ok()) {
    rclcpp::spin_some(mInternals->mNodePointer);
  }
}

// void vtkMRMLROS2NodeNode::SetAndObserveSubscriberNode(const char* nodeId){
//
//   const char* currentNodeId=this->GetNodeReferenceID("HELLO");
//   if (nodeId!=NULL && currentNodeId!=NULL && strcmp(nodeId,currentNodeId)==0)
//   {
//     // not changed
//     return;
//   }
//   this->SetAndObserveNodeReferenceID("HELLO", nodeId);
//   this->InvokeCustomModifiedEvent(vtkCommand::ModifiedEvent);
//   this->Modified();
//   std::cerr << "ROS2 node is modified" << std::endl;
// }
