#include <vtkMRMLROS2ParameterNode.h>
#include <vtkMRMLROS2ParameterInternals.h>

vtkStandardNewMacro(vtkMRMLROS2ParameterNode);

// typedef vtkMRMLROS2ParameterNativeInternals<ros_type, slicer_type>	vtkMRMLROS2ParameterInternals;

void vtkMRMLROS2ParameterNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
  // Custom prints
  // os << indent << "Topic: " << mTopic << "\n";
  // os << indent << "ROS type: " << mInternals->GetROSType() << "\n";
  // os << indent << "Slicer type: " << mInternals->GetSlicerType() << "\n"; // This is scrambled
  // os << indent << "Number of messages: " << mNumberOfMessages << "\n";
  // os << indent << "Last message:" << mInternals->GetLastMessageYAML() << "\n";
}

bool vtkMRMLROS2ParameterNode::AddToROS2Node(const char * nodeId,
					      const std::string & trackedNodeName)
{
  mTopic = trackedNodeName;
  mMRMLNodeName = "ros2:par:" + trackedNodeName;
  this->SetName(mMRMLNodeName.c_str());
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkWarningMacro(<< "AddToROS2Node, parameter MRML node for trackedNodeName \"" << trackedNodeName << "\" needs to be added to the scene first");
    return false;
  }
  std::string errorMessage;
  if (!mInternals->AddToROS2Node(scene, nodeId, trackedNodeName, errorMessage)) {
    vtkErrorMacro(<< "AddToROS2Node, " << errorMessage);
    return false;
  }
  return true;
}

bool vtkMRMLROS2ParameterNode::IsAddedToROS2Node(void) const
{
  return mInternals->IsAddedToROS2Node();
}

void vtkMRMLROS2ParameterNode::WriteXML(std::ostream& of, int nIndent)
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(topicName, Topic);
  vtkMRMLWriteXMLEndMacro();
}

void vtkMRMLROS2ParameterNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(topicName, Topic);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}

void vtkMRMLROS2ParameterNode::UpdateScene(vtkMRMLScene *scene)
{
  Superclass::UpdateScene(scene);
  int nbNodeRefs = this->GetNumberOfNodeReferences("node");
  if (nbNodeRefs != 1) {
    vtkErrorMacro(<< "No ROS2 node reference defined for subscriber \"" << GetName() << "\"");
  } else {
    this->AddToROS2Node(this->GetNthNodeReference("node", 0)->GetID(),
			mTopic);
  }
}

 vtkMRMLROS2ParameterNode::vtkMRMLROS2ParameterNode()	{									
   mInternals = new vtkMRMLROS2ParameterInternals(this);	
 }									
									
 vtkMRMLROS2ParameterNode::~vtkMRMLROS2ParameterNode() {									
   delete mInternals;							
 }									
									
 vtkMRMLNode * vtkMRMLROS2ParameterNode::CreateNodeInstance(void) {									
   return SelfType::New();						
 }									
									
 const char * vtkMRMLROS2ParameterNode::GetNodeTagName(void)	{									
   return "ROS2Parameter";					
 }	