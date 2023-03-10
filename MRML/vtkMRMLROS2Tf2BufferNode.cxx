// Slicer includes
#include <vtkObject.h>
#include <vtkMRMLTransformNode.h>
#include <vtkEventBroker.h>
#include <vtkMRMLScene.h>

// SlicerROS2 includes
#include <vtkMRMLROS2Utils.h>
#include <vtkMRMLROS2Tf2BufferNode.h>
#include <vtkMRMLROS2Tf2BufferInternals.h>
#include <vtkMRMLROS2Tf2LookupNode.h>
#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2NodeInternals.h>
#include <vtkROS2ToSlicer.h>


vtkStandardNewMacro(vtkMRMLROS2Tf2BufferNode);

vtkMRMLROS2Tf2BufferNode::vtkMRMLROS2Tf2BufferNode()
{
  mInternals = std::make_unique<vtkMRMLROS2Tf2BufferInternals>();
  mTemporaryMatrix = vtkMatrix4x4::New();
}


vtkMRMLROS2Tf2BufferNode::~vtkMRMLROS2Tf2BufferNode()
{
}


vtkMRMLNode * vtkMRMLROS2Tf2BufferNode::CreateNodeInstance(void)
{
  return SelfType::New();
}


const char * vtkMRMLROS2Tf2BufferNode::GetNodeTagName(void)
{
  return "ROS2Tf2Buffer";
}


void vtkMRMLROS2Tf2BufferNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os, indent);
}


bool vtkMRMLROS2Tf2BufferNode::AddToROS2Node(const char * nodeId)
{
  this->SetName(mMRMLNodeName.c_str());

  // Check if the node is in the scene
  std::string errorMessage;
  vtkMRMLROS2NodeNode * rosNodePtr = vtkMRMLROS2::CheckROS2NodeExists(this, nodeId, errorMessage);
  if (!rosNodePtr) {
    vtkErrorMacro(<< "AddToROS2Node: " << errorMessage);
    return false;
  }

  // Add the buffer to the ros2 node
  mInternals->mNodePointer = rosNodePtr->mInternals->mNodePointer;
  vtkSmartPointer<vtkMRMLROS2Tf2BufferNode> buffer = rosNodePtr->GetTf2Buffer();
  if ((buffer != nullptr) && buffer->IsAddedToROS2Node()) {
    vtkErrorMacro(<< "AddToROS2Node on \"" << mMRMLNodeName << "\": this buffer has already been added to the ROS2 node");
    return false;
  }
  mInternals->mTfBuffer = std::make_unique<tf2_ros::Buffer>(mInternals->mNodePointer->get_clock());
  mInternals->mTfListener = std::make_shared<tf2_ros::TransformListener>(*mInternals->mTfBuffer);
  rosNodePtr->SetNthNodeReferenceID("buffer",
                                    rosNodePtr->GetNumberOfNodeReferences("buffer"),
                                    this->GetID());
  this->SetNodeReferenceID("node", nodeId);
  rosNodePtr->mTf2Buffer = this;
  return true;
}


bool vtkMRMLROS2Tf2BufferNode::IsAddedToROS2Node(void) const
{
  return (mInternals->mTfBuffer != nullptr);
}


bool vtkMRMLROS2Tf2BufferNode::AddLookupNode(vtkMRMLROS2Tf2LookupNode * lookupNode)
{
  // Check that the buffer is in the scene
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddLookupNode on \""<< mMRMLNodeName << "\": tf2 buffer MRML node needs to be added to the scene first");
    return false;
  }
  // Check if the lookup node is already in the list of lookups
  if (GetLookupNodeByID(lookupNode->GetID()) != nullptr) {
    vtkErrorMacro(<< "AddLookupNode on \"" << mMRMLNodeName << "\": this lookup node has been added to the buffer already (based on ID)");
    return false;
  }
  // Check that the parent and child of the lookup are set and then add to the buffer node and assign references
  if (lookupNode->IsParentAndChildSet()) {
    this->SetNodeReferenceID("lookups", lookupNode->GetID()); // Add both references
    lookupNode->SetNodeReferenceID("buffer", this->GetID());
    return true;
  }
  else {
    vtkErrorMacro(<< "AddLookupNode on \"" << mMRMLNodeName << "\": lookup child and parent are not set");
    return false;
  }
}


vtkMRMLROS2Tf2LookupNode * vtkMRMLROS2Tf2BufferNode::CreateAndAddLookupNode(const std::string & parent_id, const std::string & child_id)
{
  // Check the buffer node is in the scene
  if (!this->GetScene()) {
    vtkErrorMacro(<< "CreateAndAddLookupNode on \"" << mMRMLNodeName << "\": tf2 buffer MRML node needs to be added to the scene first");
    return nullptr;
  }

  // Create the lookup node
  vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookupNode = vtkMRMLROS2Tf2LookupNode::New();
  // Add the node to the scene
  this->GetScene()->AddNode(lookupNode);

  // Set the parent and child id
  if (!lookupNode->SetParentID(parent_id)) {
    vtkErrorMacro(<< "CreateAndAddLookupNode on \"" << mMRMLNodeName << "\": parent ID is an empty string");
    return nullptr;
  }
  if (!lookupNode->SetChildID(child_id)) {
    vtkErrorMacro(<< "CreateAndAddLookupNode on \"" << mMRMLNodeName << "\": child ID is an empty string");
    return nullptr;
  }

  // Add to list of lookup nodes and assigned references
  lookupNode->SetNodeReferenceID("buffer", this->GetID());

  this->SetNthNodeReferenceID("lookups",
                              this->GetNumberOfNodeReferences("lookups"),
                              lookupNode->GetID());
  return lookupNode;
}


vtkMRMLROS2Tf2LookupNode * vtkMRMLROS2Tf2BufferNode::GetLookupNodeByID(const std::string & nodeID)
{
  // Go through list of lookup nodes and check for duplicate IDs
  int nbLookupRefs = this->GetNumberOfNodeReferences("lookups");
  for (int i = 0; i < nbLookupRefs; i ++){
    vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookupNode = vtkMRMLROS2Tf2LookupNode::SafeDownCast(this->GetNthNodeReference("lookups", i));
    const std::string lookupID = lookupNode->GetID();
    if (lookupID == nodeID) {
      return lookupNode;
    }
  }
  return nullptr;
}


void vtkMRMLROS2Tf2BufferNode::Spin(void)
{
  // Make sure buffer node is still in the scene
  if (!this->GetScene()) {
    vtkErrorMacro(<< "Spin on \"" << mMRMLNodeName << "\": tf2 buffer MRML node needs to be added to the scene first");
    return;
  }
  // iterate through lookup nodes - make sure they have parent and children set and call lookup try catch
  int nbLookupRefs = this->GetNumberOfNodeReferences("lookups");
  for (int i = 0; i < nbLookupRefs; i ++){
    vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookupNode = vtkMRMLROS2Tf2LookupNode::SafeDownCast(this->GetNthNodeReference("lookups", i));
    const std::string & parent_id = lookupNode->GetParentID();
    const std::string & child_id = lookupNode->GetChildID();
    try {
      geometry_msgs::msg::TransformStamped transformStamped;
      // check how old we want the data to be (right now it's doing it no matter how old) - for now we don't care
      transformStamped = mInternals->mTfBuffer->lookupTransform(parent_id, child_id, tf2::TimePointZero);
      vtkROS2ToSlicer(transformStamped, mTemporaryMatrix);
      if (lookupNode->GetModifiedOnLookup()) {
        lookupNode->SetMatrixTransformToParent(mTemporaryMatrix);
      } else {
        lookupNode->DisableModifiedEventOn();
        lookupNode->SetMatrixTransformToParent(mTemporaryMatrix);
        lookupNode->DisableModifiedEventOff();
      }
    }
    catch (tf2::TransformException & ex) {
      vtkErrorMacro(<< "Spin on \"" << mMRMLNodeName << ": could not find the transform between " << parent_id << " and " << child_id << ", " << ex.what());
    }
    catch (...) {
      vtkErrorMacro(<< "Spin on \"" << mMRMLNodeName << ": undefined exception while looking up transform between " << parent_id << " and " << child_id);
    }
  }
}


void vtkMRMLROS2Tf2BufferNode::WriteXML(ostream& of, int nIndent)
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(BufferNodeName, BufferNodeName);
  vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2Tf2BufferNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(BufferNodeName, BufferNodeName);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}


void vtkMRMLROS2Tf2BufferNode::UpdateScene(vtkMRMLScene *scene)
{
  Superclass::UpdateScene(scene);
  if (!IsAddedToROS2Node()) {
    int nbNodeRefs = this->GetNumberOfNodeReferences("node");
    if (nbNodeRefs != 1) {
      vtkErrorMacro(<< "No ROS2 node reference defined for buffer \"" << GetName() << "\"");
    } else {
      this->AddToROS2Node(this->GetNthNodeReference("node", 0)->GetID());
    }
  }
}
