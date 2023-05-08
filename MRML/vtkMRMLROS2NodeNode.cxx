#include <vtkMRMLROS2NodeNode.h>

#include <vtkMatrix4x4.h>
#include <vtkMRMLScene.h>

#include <vtkROS2ToSlicer.h>
#include <vtkMRMLROS2NodeInternals.h>
#include <vtkMRMLROS2SubscriberNode.h>
#include <vtkMRMLROS2PublisherNode.h>
#include <vtkMRMLROS2ParameterNode.h>
#include <vtkMRMLROS2Tf2BroadcasterNode.h>
#include <vtkMRMLROS2Tf2LookupNode.h>
#include <vtkMRMLROS2RobotNode.h>
#include <vtkMRMLModelNode.h>

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
  mTemporaryMatrix = vtkMatrix4x4::New();
}


vtkMRMLROS2NodeNode::~vtkMRMLROS2NodeNode()
{
  // this->Destroy(); // FIXME: This causes a recursive call to Destroy()
}


void vtkMRMLROS2NodeNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os, indent);
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
  mROS2NodeName = "undefined";
  mMRMLNodeName = "ros2:node:undefined";
  this->SetName(mMRMLNodeName.c_str());
  this->Scene->RemoveNode(this);
  mInternals->mNodePointer.reset();
  mInternals.reset();
}


vtkMRMLROS2SubscriberNode * vtkMRMLROS2NodeNode::CreateAndAddSubscriberNode(const char * className, const std::string & topic)
{
  // Check if this has been added to the scene
  if (this->GetScene() == nullptr) {
    vtkErrorMacro(<< "CreateAndAddSubscriber: \"" << mROS2NodeName << "\" must be added to a MRML scene first");
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


vtkMRMLROS2PublisherNode * vtkMRMLROS2NodeNode::CreateAndAddPublisherNode(const char * className, const std::string & topic)
{
  // Check if this has been added to the scene
  if (this->GetScene() == nullptr) {
    vtkErrorMacro(<< "CreateAndAddPublisher: \"" << mROS2NodeName << "\" must be added to a MRML scene first");
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


vtkMRMLROS2ParameterNode * vtkMRMLROS2NodeNode::CreateAndAddParameterNode(const std::string & monitoredNodeName)
{
  // Check if this has been added to the scene
  if (this->GetScene() == nullptr) {
    vtkErrorMacro(<< "CreateAndAddParameter: \"" << mROS2NodeName << "\" must be added to a MRML scene first");
    return nullptr;
  }
  // CreateNodeByClass
  vtkSmartPointer<vtkMRMLROS2ParameterNode> parameterNode = vtkMRMLROS2ParameterNode::New();
  // Add to the scene so the ROS2Node node can find it
  this->GetScene()->AddNode(parameterNode);
  if (parameterNode->AddToROS2Node(this->GetID(), monitoredNodeName)) {
    return parameterNode;
  }
  // Something went wrong, cleanup
  this->GetScene()->RemoveNode(parameterNode);
  parameterNode->Delete();
  return nullptr;
}


vtkMRMLROS2Tf2BroadcasterNode * vtkMRMLROS2NodeNode::CreateAndAddTf2BroadcasterNode(const std::string & parent_id, const std::string & child_id)
{
  // Check if this has been added to the scene
  if (this->GetScene() == nullptr) {
    vtkErrorMacro(<< "CreateAndAddTf2Broadcaster: \"" << mROS2NodeName << "\" must be added to a MRML scene first");
    return nullptr;
  }
  // Create the broadcaster node
  vtkSmartPointer<vtkMRMLROS2Tf2BroadcasterNode> broadcasterNode = vtkMRMLROS2Tf2BroadcasterNode::New();
  // Add to the scene so the ROS2Node node can find it
  this->GetScene()->AddNode(broadcasterNode);
  if (broadcasterNode->AddToROS2Node(this->GetID())) {
    broadcasterNode->SetParentID(parent_id);
    broadcasterNode->SetChildID(child_id);
    return broadcasterNode;
  }
  // Something went wrong, cleanup
  this->GetScene()->RemoveNode(broadcasterNode);
  broadcasterNode->Delete();
  return nullptr;
}


vtkMRMLROS2Tf2LookupNode * vtkMRMLROS2NodeNode::CreateAndAddTf2LookupNode(const std::string & parent_id, const std::string & child_id)
{
  // Check the buffer node is in the scene
  if (this->GetScene() == nullptr) {
    vtkErrorMacro(<< "CreateAndAddTf2LookupNode: \"" << mROS2NodeName << "\" must be added to a MRML scene first");
    return nullptr;
  }

  // Create the lookup node
  vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookupNode = vtkMRMLROS2Tf2LookupNode::New();
  // Add the node to the scene
  this->GetScene()->AddNode(lookupNode);
  if (lookupNode->AddToROS2Node(this->GetID())) {
    lookupNode->SetParentID(parent_id);
    lookupNode->SetChildID(child_id);
    return lookupNode;
  }
  // Something went wrong, cleanup
  this->GetScene()->RemoveNode(lookupNode);
  lookupNode->Delete();
  return nullptr;
}


vtkMRMLROS2RobotNode * vtkMRMLROS2NodeNode::CreateAndAddRobotNode(const std::string & robotName, const std::string & parameterNodeName, const std::string & parameterName)
{
  // Check if this has been added to the scene
  if (this->GetScene() == nullptr) {
    vtkErrorMacro(<< "CreateAndAddRobotNode: \"" << mROS2NodeName << "\" must be added to a MRML scene first");
    return nullptr;
  }

  // Create the robot node
  vtkSmartPointer<vtkMRMLROS2RobotNode> robotNode = vtkMRMLROS2RobotNode::New();

  // Add it to the scene
  this->GetScene()->AddNode(robotNode);
  robotNode->AddToROS2Node(this->GetID(), robotName, parameterNodeName, parameterName);
  return robotNode;
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


vtkMRMLROS2ParameterNode* vtkMRMLROS2NodeNode::GetParameterNodeByNodeID(const std::string & nodeID)
{
  size_t parameterRefs = this->GetNumberOfNodeReferences("parameter");
  for (size_t j = 0; j < parameterRefs; ++j) {
    vtkMRMLROS2ParameterNode * node = vtkMRMLROS2ParameterNode::SafeDownCast(this->GetNthNodeReference("parameter", j));
    if (!node) {
      vtkWarningMacro(<< "GetParameterNodeByNodeID: node referenced by role 'parameter' is not a parameter.");
    } else if (nodeID == node->GetID()) {
      return node;
    }
  }
  return nullptr; // otherwise return a null ptr
}


vtkMRMLROS2Tf2BroadcasterNode * vtkMRMLROS2NodeNode::GetTf2BroadcasterNodeByID(const std::string & nodeID)
{
  size_t broadcasterRefs = this->GetNumberOfNodeReferences("broadcaster");
  for (size_t j = 0; j < broadcasterRefs; ++j) {
    vtkSmartPointer<vtkMRMLROS2Tf2BroadcasterNode> node = vtkMRMLROS2Tf2BroadcasterNode::SafeDownCast(this->GetNthNodeReference("broadcaster", j));
    if (!node) {
      vtkWarningMacro(<< "GetTf2BroadcasterNodeByID: node referenced by role 'broadcaster' is not a broadcaster");
    } else if (node->GetID() == nodeID) {
      return node;
    }
  }
  return nullptr; // otherwise return a null ptr
}


vtkMRMLROS2Tf2BroadcasterNode * vtkMRMLROS2NodeNode::GetTf2BroadcasterNodeByParentChild(const std::string & parent_id, const std::string & child_id)
{
  size_t broadcasterRefs = this->GetNumberOfNodeReferences("broadcaster");
  for (size_t j = 0; j < broadcasterRefs; ++j) {
    vtkSmartPointer<vtkMRMLROS2Tf2BroadcasterNode> node = vtkMRMLROS2Tf2BroadcasterNode::SafeDownCast(this->GetNthNodeReference("broadcaster", j));
    if (!node) {
      vtkWarningMacro(<< "GetTf2BroadcasterNodeByParentChild: node referenced by role 'broadcaster' is not a broadcaster");
    } else if (node->GetParentID() == parent_id && node->GetChildID() == child_id) {
      return node;
    }
  }
  return nullptr; // otherwise return a null ptr
}


vtkMRMLROS2Tf2LookupNode * vtkMRMLROS2NodeNode::GetTf2LookupNodeByID(const std::string & nodeID)
{
  size_t lookupRefs = this->GetNumberOfNodeReferences("lookup");
  for (size_t j = 0; j < lookupRefs; ++j) {
    vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> node = vtkMRMLROS2Tf2LookupNode::SafeDownCast(this->GetNthNodeReference("lookup", j));
    if (!node) {
      vtkWarningMacro(<< "GetTf2LookupNodeByID: node referenced by role 'lookup' is not a lookup");
    } else if (node->GetID() == nodeID) {
      return node;
    }
  }
  return nullptr; // otherwise return a null ptr
}


vtkMRMLROS2Tf2LookupNode * vtkMRMLROS2NodeNode::GetTf2LookupNodeByParentChild(const std::string & parent_id, const std::string & child_id)
{
  size_t lookupRefs = this->GetNumberOfNodeReferences("lookup");
  for (size_t j = 0; j < lookupRefs; ++j) {
    vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> node = vtkMRMLROS2Tf2LookupNode::SafeDownCast(this->GetNthNodeReference("lookup", j));
    if (!node) {
      vtkWarningMacro(<< "GetTf2LookupNodeByParentChild: node referenced by role 'lookup' is not a lookup");
    } else if (node->GetParentID() == parent_id && node->GetChildID() == child_id) {
      return node;
    }
  }
  return nullptr; // otherwise return a null ptr
}

vtkMRMLROS2RobotNode * vtkMRMLROS2NodeNode::GetRobotNodeByName(const std::string & robotName)
{
  size_t robotRefs = this->GetNumberOfNodeReferences("robot");
  for (size_t j = 0; j < robotRefs; ++j) {
    vtkSmartPointer<vtkMRMLROS2RobotNode> node = vtkMRMLROS2RobotNode::SafeDownCast(this->GetNthNodeReference("robot", j));
    if (!node) {
      vtkWarningMacro(<< "GetRobotNodeByName: node referenced by role 'robot' is not a robot");
    } else if (node->GetRobotName() == robotName) {
      return node;
    }
  }
  return nullptr; // otherwise return a null ptr
}


bool vtkMRMLROS2NodeNode::RemoveAndDeleteSubscriberNode(const std::string & topic)
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


bool vtkMRMLROS2NodeNode::RemoveAndDeletePublisherNode(const std::string & topic)
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


bool vtkMRMLROS2NodeNode::RemoveAndDeleteParameterNode(const std::string & monitoredNodeName)
{
  vtkMRMLROS2ParameterNode * node = this->GetParameterNodeByNode(monitoredNodeName);
  if (!node) {
    vtkWarningMacro(<< "RemoveParameterNode: node referenced by role 'parameter' does not exist");
    return false;
  }
  node->RemoveFromROS2Node(this->GetID());
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return true;
}


bool vtkMRMLROS2NodeNode::RemoveAndDeleteParameterNodeByNodeID(const std::string & nodeID)
{
  vtkMRMLROS2ParameterNode * node = this->GetParameterNodeByNodeID(nodeID);
  if (!node) {
    vtkWarningMacro(<< "RemoveParameterNode: node referenced by role 'parameter' does not exist");
    return false;
  }
  node->RemoveFromROS2Node(this->GetID());
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return true;
}


bool vtkMRMLROS2NodeNode::RemoveAndDeleteTf2LookupNode(const std::string & nodeID)
{
  vtkMRMLROS2Tf2LookupNode * node = this->GetTf2LookupNodeByID(nodeID);
  if (!node) {
    vtkWarningMacro(<< "RemoveTf2LookupNode: node referenced by role 'lookup' for node " << nodeID << " does not exist");
    return false;
  }
  node->RemoveFromROS2Node(this->GetID());
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return true;
}


bool vtkMRMLROS2NodeNode::RemoveAndDeleteTf2LookupNode(const std::string & parent_id, const std::string & child_id)
{
  vtkMRMLROS2Tf2LookupNode * node = this->GetTf2LookupNodeByParentChild(parent_id, child_id);
  if (!node) {
    vtkWarningMacro(<< "RemoveTf2LookupNode: node referenced by role 'lookup' for node " << parent_id << " and " << child_id << " does not exist");
    return false;
  }
  node->RemoveFromROS2Node(this->GetID());
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return true;
}


bool vtkMRMLROS2NodeNode::RemoveAndDeleteTf2BroadcasterNode(const std::string & nodeID)
{
  vtkMRMLROS2Tf2BroadcasterNode * node = this->GetTf2BroadcasterNodeByID(nodeID);
  if (!node) {
    vtkWarningMacro(<< "RemoveTf2BroadcasterNode: node referenced by role 'broadcaster' for node " << nodeID << " does not exist");
    return false;
  }
  node->RemoveFromROS2Node(this->GetID());
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return true;
}


bool vtkMRMLROS2NodeNode::RemoveAndDeleteTf2BroadcasterNode(const std::string & parent_id, const std::string & child_id)
{
  vtkMRMLROS2Tf2BroadcasterNode * node = this->GetTf2BroadcasterNodeByParentChild(parent_id, child_id);
  if (!node) {
    vtkWarningMacro(<< "RemoveTf2BroadcasterNode: node referenced by role 'broadcaster' for node " << parent_id << " and " << child_id << " does not exist");
    return false;
  }
  node->RemoveFromROS2Node(this->GetID());
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return true;
}


bool vtkMRMLROS2NodeNode::RemoveAndDeleteRobotNode(const std::string & robotName)
{
  vtkMRMLROS2RobotNode * node = this->GetRobotNodeByName(robotName);
  if (!node) {
    vtkWarningMacro(<< "RemoveAndDeleteRobotNode: node referenced by role 'robot' for node " << robotName << " does not exist");
    return false;
  }

  // Remove the lookups on that robot
  int numLookups = node->GetNumberOfNodeReferences("lookup");
  for (int i = 0; i < numLookups; i++) {
    auto lookupNodeID = node->GetNthNodeReferenceID("lookup", 0); // always grab the first one because the ref id changes
    auto modelNode = vtkMRMLModelNode::SafeDownCast(node->GetNthNodeReference("model", 0)); // always grab the first one because the ref id changes
    this->RemoveAndDeleteTf2LookupNode(lookupNodeID);
    this->GetScene()->RemoveNode(modelNode);
    modelNode->Delete();
  }

  auto parameterNodeID = node->GetNthNodeReferenceID("parameter", 0); // always grab the first one because the ref id changes
  this->RemoveAndDeleteParameterNodeByNodeID(parameterNodeID);

  // Remove the robot itself
  this->GetScene()->RemoveNode(node);
  node->Delete();
  return true;
}


bool vtkMRMLROS2NodeNode::SetTf2Buffer(void)
{
  // if it exists
  if (mInternals->mTf2Buffer != nullptr) {
    return true;
  }
  // else try to create all internals if we have a proper ros node
  if (mInternals->mNodePointer != nullptr) {
    mInternals->mTf2Buffer = std::make_unique<tf2_ros::Buffer>(mInternals->mNodePointer->get_clock());
    mInternals->mTf2Listener = std::make_shared<tf2_ros::TransformListener>(*mInternals->mTf2Buffer);
    return true;
  } else {
    vtkWarningMacro(<< "SetTf2Buffer: trying to setup the tf2 buffer before the ROS internal node has been created for \"" << GetName() << "\"");
    return false;
  }
  return false;
}


void vtkMRMLROS2NodeNode::SpinTf2Buffer(void)
{
  if (mInternals->mTf2Buffer != nullptr) {
    // iterate through lookup nodes - make sure they have parent and children set and call lookup try catch
    int nbLookupRefs = this->GetNumberOfNodeReferences("lookup");
    for (int i = 0; i < nbLookupRefs; i ++) {
      vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookupNode = vtkMRMLROS2Tf2LookupNode::SafeDownCast(this->GetNthNodeReference("lookup", i));
      const std::string & parent_id = lookupNode->GetParentID();
      const std::string & child_id = lookupNode->GetChildID();
      try {
        geometry_msgs::msg::TransformStamped transformStamped;
        // check how old we want the data to be (right now it's doing it no matter how old) - for now we don't care
        transformStamped = mInternals->mTf2Buffer->lookupTransform(parent_id, child_id, tf2::TimePointZero);
        if (lookupNode->IsDifferentFromLast(transformStamped.header.stamp.sec, transformStamped.header.stamp.nanosec)) {
          vtkROS2ToSlicer(transformStamped, mTemporaryMatrix);
          if (lookupNode->GetModifiedOnLookup()) {
            lookupNode->SetMatrixTransformToParent(mTemporaryMatrix);
          } else {
            lookupNode->DisableModifiedEventOn();
            lookupNode->SetMatrixTransformToParent(mTemporaryMatrix);
            lookupNode->DisableModifiedEventOff();
          }
        }
      }
      catch (tf2::TransformException & ex) {
        vtkErrorMacro(<< "SpinTf2Buffer on \"" << mMRMLNodeName << ": could not find the transform between " << parent_id << " and " << child_id << ", " << ex.what());
      }
      catch (...) {
        vtkErrorMacro(<< "SpinTf2Buffer on \"" << mMRMLNodeName << ": undefined exception while looking up transform between " << parent_id << " and " << child_id);
      }
    }
  }
}


void vtkMRMLROS2NodeNode::Spin(void)
{
  if (rclcpp::ok()) {
    mSpinning = true;
    // for all ROS callbacks
    rclcpp::spin_some(mInternals->mNodePointer);
    // parameters
    for (auto & node : this->mParameterNodes) {
      if (node != nullptr) {
        node->Spin();
      }
    }
    // tf2 lookups / buffer
    SpinTf2Buffer();
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
