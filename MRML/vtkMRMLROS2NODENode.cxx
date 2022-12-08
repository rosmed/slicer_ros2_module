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

void vtkMRMLROS2NODENode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
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
  mROS2NodeName = nodeName;
  mMRMLNodeName = "ros2:node:" + nodeName;
  this->SetName(mMRMLNodeName.c_str());
  mInternals->mNodePointer = std::make_shared<rclcpp::Node>(nodeName);
}

vtkMRMLROS2SubscriberNode* vtkMRMLROS2NODENode::GetSubscriberNodeByTopic(const std::string & topic){
    
  int subscriberRefs = this->GetNumberOfNodeReferences("subscriber");
  for (int j = 0; j < subscriberRefs; j ++){
    
    vtkMRMLROS2SubscriberNode * node = vtkMRMLROS2SubscriberNode::SafeDownCast(this->GetNthNodeReference("subscriber", j));
    if (!node){
      vtkWarningMacro(<< "Node referenced by role 'subscriber' is not a subscriber");
    }
    std::string topicName = node->GetTopic(); 
    if (topicName == topic){ // check if an existing nodes name matches the topic provided
      return node; // if so return the node
    }
  }
  return nullptr; // otherwise return a null ptr
}

vtkMRMLROS2PublisherNode* vtkMRMLROS2NODENode::GetPublisherNodeByTopic(const std::string & topic){
    
  int publisherRefs = this->GetNumberOfNodeReferences("publisher");
  for (int j = 0; j < publisherRefs; j ++){
    vtkMRMLROS2PublisherNode * node = vtkMRMLROS2PublisherNode::SafeDownCast(this->GetNthNodeReference("publisher", j));
    if (!node){
      vtkWarningMacro(<< "Node referenced by role 'subscriber' is not a subscriber");
    }
    std::string topicName = node->GetTopic(); 
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


void vtkMRMLROS2NODENode::WriteXML( ostream& of, int nIndent )
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkIndent indent(nIndent);

  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(ROS2NodeName, mROS2NodeName);
  vtkMRMLWriteXMLEndMacro();
}

//------------------------------------------------------------------------------
void vtkMRMLROS2NODENode::ReadXMLAttributes( const char** atts )
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(ROS2NodeName, mROS2NodeName);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);

  // This is created before UpdateScene() for all other nodes is called.
  // It handles cases where Publishers and Subscribers are Read before the ROS2Node
  this->Create(mROS2NodeName,false);
}

void vtkMRMLROS2NODENode::UpdateScene(vtkMRMLScene *scene)
{
    Superclass::UpdateScene(scene);
    std::cerr << "ROS2NODENode updated" << std::endl;
}