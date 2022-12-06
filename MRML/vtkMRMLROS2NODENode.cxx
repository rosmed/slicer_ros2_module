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

vtkMRMLNode* vtkMRMLROS2NODENode::GetSubscriberNodeByTopic(const std::string & topic){
    
  int subscriberRefs = this->GetNumberOfNodeReferences("subscriber");
  for (int j = 0; j < subscriberRefs; j ++){
    vtkMRMLNode * node = this->GetNthNodeReference("subscriber", j);
    std::string nodeName = node->GetName(); // the node name has format "ros2:sub:/topicname"
    std::string delimiter = ":"; // Split by this 
    std::string substring = nodeName.substr((nodeName.find(delimiter)+1), nodeName.length()); // break the node name into the second half sub:/topicname
    std::string topicName = substring.substr((substring.find(delimiter)+1), substring.length()); // break the node name again into just /topicname
    if (topicName == topic){ // check if an existing nodes name matches the topic provided
      return node; // if so return the node
    }
  }
  return nullptr; // otherwise return a null ptr
}

vtkMRMLNode* vtkMRMLROS2NODENode::GetPublisherNodeByTopic(const std::string & topic){
    
  int publisherRefs = this->GetNumberOfNodeReferences("publisher");
  for (int j = 0; j < publisherRefs; j ++){
    vtkMRMLNode * node = this->GetNthNodeReference("publisher", j);
    std::string nodeName = node->GetName(); // the node name has format "ros2:sub:/topicname"
    std::string delimiter = ":"; // Split by this 
    std::string substring = nodeName.substr((nodeName.find(delimiter)+1), nodeName.length()); // break the node name into the second half sub:/topicname
    std::string topicName = substring.substr((substring.find(delimiter)+1), substring.length()); // break the node name again into just /topicname
    if (topicName == topic){ // check if an existing nodes name matches the topic provided
      return node; // if so return the node
    }
  }
  return nullptr; // otherwise return a null ptr
}

void vtkMRMLROS2NODENode::Spin(void)
{
  if (rclcpp::ok()) {
    rclcpp::spin_some(mInternals->mNodePointer);
  }
}
