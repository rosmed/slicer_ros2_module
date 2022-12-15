#include <vtkMRMLROS2Tf2BroadcasterNode.h>
#include <vtkMRMLROS2Tf2BroadcasterInternals.h>
#include "vtkCommand.h"
#include <vtkSlicerToROS2.h>

vtkStandardNewMacro(vtkMRMLROS2Tf2BroadcasterNode);

vtkMRMLNode * vtkMRMLROS2Tf2BroadcasterNode::CreateNodeInstance(void)
{
  return SelfType::New();

}


const char * vtkMRMLROS2Tf2BroadcasterNode::GetNodeTagName(void)
{
  return "ROS2Tf2Broadcaster";
}


vtkMRMLROS2Tf2BroadcasterNode::vtkMRMLROS2Tf2BroadcasterNode()
{
  mInternals = std::make_unique<vtkMRMLROS2Tf2BroadcasterInternals>();
}

vtkMRMLROS2Tf2BroadcasterNode::~vtkMRMLROS2Tf2BroadcasterNode()
{
}


void vtkMRMLROS2Tf2BroadcasterNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
}

bool vtkMRMLROS2Tf2BroadcasterNode::AddToROS2Node(const char * nodeId)
{
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 broadcaster MRML node needs to be added to the scene first");
    return false;
  }
  std::string errorMessage;
  if (!mInternals->AddToROS2Node(scene, nodeId, errorMessage)) {
    vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
    return false;
  }
  return true;
}

void vtkMRMLROS2Tf2BroadcasterNode::Create(const std::string & nodeName, bool initialize)
{
  // - this should be detected automatically by look for node of type ROS2Node in scene
  // - there might also be a rclcpp method to detect if the context has been initialized
  // if (initialize) {
  //   typedef char * char_pointer;
  //   char_pointer * argv = new char_pointer[1];
  //   argv[0]= new char[nodeName.size() + 1];
  //   strcpy(argv[0], nodeName.c_str());
  //   int argc = 1;
  //   rclcpp::init(argc, argv);
  // }

  // create the ROS node
  // mROS2NodeName = nodeName;
  // mMRMLNodeName = "ros2:node:" + nodeName;
  // this->SetName(mMRMLNodeName.c_str());
  // mInternals->mNodePointer = std::make_shared<rclcpp::Node>(nodeName);
}




void vtkMRMLROS2Tf2BroadcasterNode::WriteXML( ostream& of, int nIndent )
{
  // Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  // vtkIndent indent(nIndent);

  // vtkMRMLWriteXMLBeginMacro(of);
  // vtkMRMLWriteXMLStdStringMacro(ROS2NodeName, ROS2NodeName);
  // vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2Tf2BroadcasterNode::ReadXMLAttributes( const char** atts )
{
  // int wasModifying = this->StartModify();
  // Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  // vtkMRMLReadXMLBeginMacro(atts);
  // vtkMRMLReadXMLStdStringMacro(ROS2NodeName, ROS2NodeName);
  // vtkMRMLReadXMLEndMacro();
  // this->EndModify(wasModifying);

  // // This is created before UpdateScene() for all other nodes is called.
  // // It handles cases where Publishers and Subscribers are Read before the ROS2Node
  // this->Create(mROS2NodeName,false);
}
