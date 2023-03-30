#include <vtkMRMLScene.h>
#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2NodeInternals.h>
#include <vtkMRMLROS2SubscriberNode.h>
#include <vtkMRMLROS2PublisherNode.h>
#include <vtkMRMLROS2ParameterNode.h>
#include <vtkMRMLROS2Tf2BroadcasterNode.h>
#include <vtkMRMLROS2Tf2BufferNode.h>

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
  // this->Destroy(); // FIXME: This causes a recursive call to Destroy()
}


void vtkMRMLROS2NodeNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
}


void vtkMRMLROS2NodeNode::Create(const std::string & nodeName)
{
  // create the ROS node
  mROS2NodeName = nodeName;
  mMRMLNodeName = "ros2:node:" + nodeName;
  this->SetName(mMRMLNodeName.c_str());
  mInternals->mNodePointer = std::make_shared<rclcpp::Node>(nodeName);
}


void vtkMRMLROS2NodeNode::Destroy()
{

  if (!mInternals || !mInternals->mNodePointer ) {
    vtkWarningMacro(<< "Destroy: node does not contain any ROS2 internals. Not destroying ROS2 node.");
    return;
  }
  mROS2NodeName = "undefined"; // FIXME: this is a hack to prevent half destroyed nodes from being spun
  mMRMLNodeName = "ros2:node:undefined";
  this->SetName(mMRMLNodeName.c_str());
  this->Scene->RemoveNode(this);
  mInternals->mNodePointer.reset();
  mInternals.reset();
}


vtkMRMLROS2SubscriberNode * vtkMRMLROS2NodeNode::CreateAndAddSubscriber(const char * className, const std::string & topic)
{
  // Check if this has been added to the scene
  if (this->GetScene() == nullptr) {
    vtkErrorMacro(<< "CreateAndAddSubscriber: \"" << mROS2NodeName << "\" is not added to a MRML scene yet");
    return nullptr;
  }
  // CreateNodeByClass
  vtkSmartPointer<vtkMRMLNode> node = this->GetScene()->CreateNodeByClass(className);
  // Check that this is a subscriber so we can add it
  vtkMRMLROS2SubscriberNode * subscriberNode = vtkMRMLROS2SubscriberNode::SafeDownCast(node);
  if (subscriberNode == nullptr) {
    vtkErrorMacro(<< "CreateAndAddSubscriber: \"" << className << "\" is not derived from vtkMRMLROS2SubscriberNode");
    return nullptr;
  }
  // Add to the scene so the ROS2Node node can find it
  this->GetScene()->AddNode(subscriberNode);
  if (subscriberNode->AddToROS2Node(this->GetID(), topic)) {
    return subscriberNode;
  }
  // Something went wrong, cleanup
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return nullptr;
}


vtkMRMLROS2PublisherNode * vtkMRMLROS2NodeNode::CreateAndAddPublisher(const char * className, const std::string & topic)
{
  // Check if this has been added to the scene
  if (this->GetScene() == nullptr) {
    vtkErrorMacro(<< "CreateAndAddPublisher: \"" << mROS2NodeName << "\" is not added to a MRML scene yet");
    return nullptr;
  }
  // CreateNodeByClass
  vtkSmartPointer<vtkMRMLNode> node = this->GetScene()->CreateNodeByClass(className);
  // Check that this is a publisher so we can add it
  vtkMRMLROS2PublisherNode * publisherNode = vtkMRMLROS2PublisherNode::SafeDownCast(node);
  if (publisherNode == nullptr) {
    vtkErrorMacro(<< "CreateAndAddPublisher: \"" << className << "\" is not derived from vtkMRMLROS2PublisherNode");
    return nullptr;
  }
  // Add to the scene so the ROS2Node node can find it
  this->GetScene()->AddNode(publisherNode);
  if (publisherNode->AddToROS2Node(this->GetID(), topic)) {
    return publisherNode;
  }
  // Something went wrong, cleanup
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return nullptr;
}


vtkMRMLROS2ParameterNode * vtkMRMLROS2NodeNode::CreateAndAddParameter(const std::string & monitoredNodeName)
{
  const char * className = "vtkMRMLROS2ParameterNode";
  // Check if this has been added to the scene
  if (this->GetScene() == nullptr) {
    vtkErrorMacro(<< "CreateAndAddParameter: \"" << mROS2NodeName << "\" is not added to a MRML scene yet");
    return nullptr;
  }
  // CreateNodeByClass
  vtkSmartPointer<vtkMRMLNode> node = this->GetScene()->CreateNodeByClass(className);
  // Check that this is a Parameter so we can add it
  vtkMRMLROS2ParameterNode * parameterNode = vtkMRMLROS2ParameterNode::SafeDownCast(node);
  if (parameterNode == nullptr) {
    vtkErrorMacro(<< "CreateAndAddParameter: \"" << className << "\" is not derived from vtkMRMLROS2ParameterNode");
    return nullptr;
  }
  // Add to the scene so the ROS2Node node can find it
  this->GetScene()->AddNode(parameterNode);
  if (parameterNode->AddToROS2Node(this->GetID(), monitoredNodeName)) {
    return parameterNode;
  }
  // Something went wrong, cleanup
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return nullptr;
}


vtkMRMLROS2Tf2BroadcasterNode * vtkMRMLROS2NodeNode::CreateAndAddTf2Broadcaster(const char * className, const std::string & parent_id, const std::string & child_id)
{
  // Check if this has been added to the scene
  if (this->GetScene() == nullptr) {
    vtkErrorMacro(<< "CreateAndAddTf2Broadcaster: \"" << mROS2NodeName << "\" is not added to a MRML scene yet");
    return nullptr;
  }
  // CreateNodeByClass
  vtkSmartPointer<vtkMRMLNode> node = this->GetScene()->CreateNodeByClass(className);
  // Check that this is a subscriber so we can add it
  vtkMRMLROS2Tf2BroadcasterNode * broadcasterNode = vtkMRMLROS2Tf2BroadcasterNode::SafeDownCast(node);
  if (broadcasterNode == nullptr) {
    vtkErrorMacro(<< "CreateAndAddTf2Broadcaster: \"" << className << "\" is not derived from vtkMRMLROS2Tf2BroadcasterNode");
    return nullptr;
  }
  // Add to the scene so the ROS2Node node can find it
  this->GetScene()->AddNode(broadcasterNode);
  if (broadcasterNode->AddToROS2Node(this->GetID())) {
    broadcasterNode->SetParentID(parent_id);
    broadcasterNode->SetChildID(child_id);
    return broadcasterNode;
  }
  // Something went wrong, cleanup
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return nullptr;
}


vtkMRMLROS2SubscriberNode * vtkMRMLROS2NodeNode::GetSubscriberNodeByTopic(const std::string & topic)
{
  size_t subscriberRefs = this->GetNumberOfNodeReferences("subscriber");
  for (size_t j = 0; j < subscriberRefs; ++j) {
    vtkMRMLROS2SubscriberNode * node = vtkMRMLROS2SubscriberNode::SafeDownCast(this->GetNthNodeReference("subscriber", j));
    if (!node) {
      vtkWarningMacro(<< "GetSubscriberNodeByTopic: node referenced by role 'subscriber' is not a subscriber");
    } else if (node->GetTopic() == topic) {
      return node;
    }
  }
  return nullptr;
}


vtkMRMLROS2PublisherNode* vtkMRMLROS2NodeNode::GetPublisherNodeByTopic(const std::string & topic)
{
  size_t publisherRefs = this->GetNumberOfNodeReferences("publisher");
  for (size_t j = 0; j < publisherRefs; ++j) {
    vtkMRMLROS2PublisherNode * node = vtkMRMLROS2PublisherNode::SafeDownCast(this->GetNthNodeReference("publisher", j));
    if (!node) {
      vtkWarningMacro(<< "GetPublisherNodeByTopic: node referenced by role 'publisher' is not a publisher");
    } else if (node->GetTopic() == topic) {
      return node;
    }
  }
  return nullptr;
}


vtkMRMLROS2ParameterNode* vtkMRMLROS2NodeNode::GetParameterNodeByNode(const std::string & nodeName)
{
  size_t parameterRefs = this->GetNumberOfNodeReferences("parameter");
  for (size_t j = 0; j < parameterRefs; ++j) {
    vtkMRMLROS2ParameterNode * node = vtkMRMLROS2ParameterNode::SafeDownCast(this->GetNthNodeReference("parameter", j));
    if (!node) {
      vtkWarningMacro(<< "GetParameterNodeByNode: node referenced by role 'parameter' is not a parameter");
    } else if (node->GetMonitoredNodeName() == nodeName) {
      return node;
    }
  }
  return nullptr; // otherwise return a null ptr
}


bool vtkMRMLROS2NodeNode::RemoveSubscriberNode(const std::string & topic)
{
  vtkMRMLROS2SubscriberNode * node = this->GetSubscriberNodeByTopic(topic);
  if (!node) {
    vtkWarningMacro(<< "RemoveSubscriberNode: node referenced by role 'subscriber' for topic " << topic << " does not exist");
    return false;
  }
  node->RemoveFromROS2Node(this->GetID(), topic);
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return true;
}


bool vtkMRMLROS2NodeNode::RemovePublisherNode(const std::string & topic)
{
  vtkMRMLROS2PublisherNode * node = this->GetPublisherNodeByTopic(topic);
  if (!node) {
    vtkWarningMacro(<< "RemovePublisherNode: node referenced by role 'publisher' for topic " << topic << " does not exist");
    return false;
  }
  node->RemoveFromROS2Node(this->GetID(), topic);
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return true;
}


bool vtkMRMLROS2NodeNode::RemoveParameterNode(const std::string & monitoredNodeName)
{
  vtkMRMLROS2ParameterNode * node = this->GetParameterNodeByNode(monitoredNodeName);
  if (!node) {
    vtkWarningMacro(<< "RemoveParameterNode: node referenced by role 'parameter' for node " << monitoredNodeName << " does not exist");
    return false;
  }
  node->RemoveFromROS2Node(this->GetID(), monitoredNodeName);
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return true;
}


vtkMRMLROS2Tf2BroadcasterNode * vtkMRMLROS2NodeNode::GetTf2BroadcasterByID(const std::string & nodeID)
{
  size_t broadcasterRefs = this->GetNumberOfNodeReferences("broadcaster");
  for (size_t j = 0; j < broadcasterRefs; ++j) {
    vtkSmartPointer<vtkMRMLROS2Tf2BroadcasterNode> node = vtkMRMLROS2Tf2BroadcasterNode::SafeDownCast(this->GetNthNodeReference("broadcaster", j));
    std::string bufferNodeID = node->GetID();
    if (bufferNodeID == nodeID) {
      return node;
    }
  }
  return nullptr; // otherwise return a null ptr
}


vtkMRMLROS2Tf2BufferNode * vtkMRMLROS2NodeNode::GetTf2Buffer(bool createIfNeeded)
{
  // create if needed, required and possible
  if ((mTf2Buffer == nullptr) && createIfNeeded && this->GetScene()) {
    mTf2Buffer = vtkMRMLROS2Tf2BufferNode::New();
    this->GetScene()->AddNode(mTf2Buffer);
    mTf2Buffer->AddToROS2Node(this->GetID());
  }
  return mTf2Buffer;
}


void vtkMRMLROS2NodeNode::Spin(void)
{
  if (rclcpp::ok()) {
    mSpinning = true;
    rclcpp::spin_some(mInternals->mNodePointer);
    if (mTf2Buffer != nullptr) {
      mTf2Buffer->Spin();
    }
    for (auto & node : this->mParameterNodes) {
      if (node != nullptr) {
        node->Spin();
      }
    }
  } else {
    mSpinning = false;
  }
}


void vtkMRMLROS2NodeNode::WarnIfNotSpinning(const std::string & contextMessage) const
{
  if (!mSpinning) {
    vtkWarningMacro(<< "Node " << mROS2NodeName << " is not spinning (yet) when " << contextMessage);
  }
}


void vtkMRMLROS2NodeNode::WriteXML( ostream& of, int nIndent )
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(ROS2NodeName, ROS2NodeName);
  vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2NodeNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(ROS2NodeName, ROS2NodeName);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);

  // This is created before UpdateScene() for all other nodes is called.
  // It handles cases where Publishers and Subscribers are read before the ROS2Node
  this->Create(mROS2NodeName);
}
