#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2NodeInternals.h>
#include "vtkCommand.h"

vtkStandardNewMacro(vtkMRMLROS2NODENode);

vtkMRMLNode * vtkMRMLROS2NODENode::CreateNodeInstance(void)
{
  return SelfType::New();
}

const char * vtkMRMLROS2NODENode::GetNodeTagName(void)
{
  return "ROS2Node";
}

vtkMRMLROS2NODENode::vtkMRMLROS2NODENode()
{
  mInternals = std::make_unique<vtkMRMLROS2NodeInternals>();
}

vtkMRMLROS2NODENode::~vtkMRMLROS2NODENode()
{
}

void vtkMRMLROS2NODENode::Create(const std::string & nodeName, bool initialize)
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

void vtkMRMLROS2NODENode::Spin(void)
{
  if (rclcpp::ok()) {
    rclcpp::spin_some(mInternals->mNodePointer);
  }
}
