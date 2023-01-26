#include <vtkMRMLScene.h>
#include <vtkMRMLROS2RobotNode.h>
#include <vtkMRMLROS2ParameterNode.h>
#include <vtkMRMLROS2Tf2LookupNode.h>
#include <vtkMRMLScene.h>
#include <vtkEventBroker.h>
#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2NodeInternals.h>
#include <vtkMRMLROS2RobotNodeInternals.h>
#include <vtkMRMLROS2Tf2BufferNode.h>
#include <regex>
#include <ament_index_cpp/get_package_share_directory.hpp>


vtkStandardNewMacro(vtkMRMLROS2RobotNode);

vtkMRMLNode * vtkMRMLROS2RobotNode::CreateNodeInstance(void)
{
  return SelfType::New();

}

const char * vtkMRMLROS2RobotNode::GetNodeTagName(void)
{
  return "ROS2RobotNode";
}

vtkMRMLROS2RobotNode::vtkMRMLROS2RobotNode()
{
  mInternals = std::make_unique<vtkMRMLROS2RobotNodeInternals>();
}

vtkMRMLROS2RobotNode::~vtkMRMLROS2RobotNode()
{
}

bool vtkMRMLROS2RobotNode::AddToROS2Node(const char * nodeId)
{
  this->SetName(mMRMLNodeName.c_str());

  // Check if the node is in the scene
  vtkMRMLScene * scene = this->GetScene();
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, robot MRML node needs to be added to the scene first");
    return false;
  }
  
  // Check that the ROS2 node node is in the scene and of the correct type
  vtkMRMLNode * rosNodeBasePtr = scene->GetNodeByID(nodeId);
  if (!rosNodeBasePtr) {
    vtkErrorMacro(<< "Unable to locate ros2 node in the scene");
    return false;
  }
  vtkMRMLROS2NODENode * rosNodePtr = dynamic_cast<vtkMRMLROS2NODENode *>(rosNodeBasePtr);
  if (!rosNodePtr) {
    vtkErrorMacro(<< std::string(rosNodeBasePtr->GetName()) + " doesn't seem to be a vtkMRMLROS2NODENode");
    return false;
  }

  // Add the robot to the ros2 node
  rosNodePtr->SetNthNodeReferenceID("robot",
				      rosNodePtr->GetNumberOfNodeReferences("robot"),
				      this->GetID());
  this->SetNodeReferenceID("node", nodeId);
  mROS2Node = rosNodePtr;
  return true;
}

bool vtkMRMLROS2RobotNode::SetRobotDescriptionParameterNode(vtkMRMLROS2ParameterNode * param)
{
  // Check if the node is in the scene
  if (!this->GetScene()) {
    vtkErrorMacro(<< "AddToROS2Node, tf2 buffer MRML node needs to be added to the scene first");
    return false;
  }
  mRobotDescriptionParameterNode = param;
  ObserveParameterNode(param);
  return true;
}

// robot node - create paramater
// add observer to parameter node and check when it's ready -> Call IsParameterSet() - paramater node has multiple params that it tracks
// has a callback that uses paramater (robot description)  to parse urdf
// should set t

// void vtkMRMLROS2RobotNode::PrintRobotDescription()
// {
//   std::string paramValue;
//   mRobotDescriptionParameterNode->PrintParameterValue("robot_description", paramValue);
//   std::cerr << "Parameter value:" << paramValue << std::endl;
// }

void vtkMRMLROS2RobotNode::ObserveParameterNode(vtkMRMLROS2ParameterNode * node )
{
  if (!this->GetScene()->GetNodeByID(node->GetID())){
    vtkErrorMacro(<< "Transform is not in the scene.");
    return;
  }
  node->AddObserver(vtkMRMLROS2ParameterNode::ParameterModifiedEvent, this, &vtkMRMLROS2RobotNode::ObserveParameterNodeCallback);
  this->SetAndObserveNodeReferenceID("ObservedParameter", node->GetID());
}

void vtkMRMLROS2RobotNode::ObserveParameterNodeCallback( vtkObject* caller, unsigned long, void* vtkNotUsed(callData))
{
  vtkMRMLROS2ParameterNode* parameterNode = vtkMRMLROS2ParameterNode::SafeDownCast(caller);
  if (!parameterNode)
  {
    return;
  }
  else
  {
    std::cerr << "Parameter node is modified." << std::endl; // for debugging
    ParseRobotDescription();
  }
}

bool vtkMRMLROS2RobotNode::ParseRobotDescription()
{
  // This doesn't work the first time? works the second time
  mRobotDescription = mRobotDescriptionParameterNode->GetParameterAsString("robot_description");
  std::cerr << "Robot description:" << mRobotDescription << std::endl;

  // Parser the urdf file into an urdf model - to get names of links and pos/ rpy
  if (!mInternals->mModel.initString(mRobotDescription)) {
      return false;
  }
  return true;
}

void vtkMRMLROS2RobotNode::InitializeLookupListFromURDF()
{
  // Get the names of each joint
  // Start with the root (base of the robot)
  std::shared_ptr<const urdf::Link> root = mInternals->mModel.getRoot();
  std::string root_name = root->name;
  mLinkNames.push_back(root_name);
  mLinkParentNames.push_back(root_name);
//   std::vector< std::shared_ptr< urdf::Visual > > visual_vector;
  mInternals->mVisualVector.push_back(root->visual);

  // Go through the rest of the robot and save to list
  size_t lastExplored = 0;
  while (lastExplored != mInternals->mVisualVector.size()){
    mInternals->mParentLinkPointer = mInternals->mModel.getLink(mLinkNames[lastExplored]);
    mInternals->mChildLinkPointer =  mInternals->mParentLinkPointer->child_links;

    for (std::shared_ptr<urdf::Link> i: mInternals->mChildLinkPointer) { // should I get rid of this
      mLinkNames.push_back(i->name);
      mLinkParentNames.push_back(mInternals->mParentLinkPointer->name);
      mInternals->mVisualVector.push_back(i->visual); // need to get the origin from the visual
    }
    lastExplored++;
  }

  mLookups.resize(mLinkNames.size());
  std::cerr << "Lookup list size" << mLookups.size() << std::endl;
}

void vtkMRMLROS2RobotNode::InitializeOffsets()
{
  //Get the origin and rpy
  mLinkModelFiles.resize(mInternals->mVisualVector.size());
  mInternals->mLinkOrigins.resize(mInternals->mVisualVector.size());
  // This was causing a big issue before - dvrk showing up in the wrong places
  for (size_t index = 0; index < mInternals->mVisualVector.size(); ++index) {
    std::shared_ptr<urdf::Visual> i = mInternals->mVisualVector[index];
    if (i == nullptr) {
      std::cerr << "no visual" << std::endl;
    } else {
      urdf::Pose origin;
      origin = i->origin;
      mInternals->mLinkOrigins[index] = origin;
      // Get stl file name and add it to a list of vectors for python parsing later
      std::shared_ptr<urdf::Mesh> mesh =  std::dynamic_pointer_cast<urdf::Mesh>(i->geometry);
      if (mesh != nullptr) {
        // See if the file name uses a package url
        std::string filename = mesh->filename;
        std::cerr << index << ": " << filename << std::endl;
        std::regex param_regex("^package:\\/\\/(\\w+)\\/(.*)");
        std::smatch match;
        if (std::regex_search(filename, match, param_regex)) {
          const std::string package = match[1];
          const std::string relativeFile = match[2];
          // Anton: add try/catch here in case the package is not found!
          const std::string packageShareDirectory
             = ament_index_cpp::get_package_share_directory(package);
          filename = packageShareDirectory + "/" + relativeFile;
        }
        mLinkModelFiles[index] = filename;
        std::cerr << index << ": " << filename << std::endl;
      } else {
        std::cerr << "link " << index << " has a visual, but not from file" << std::endl;
      }
    }
  }
}

void vtkMRMLROS2RobotNode::InitializeLookups()
{
  for (size_t i = 0; i < (mInternals->mVisualVector.size()); i++){
    mROS2Node->mBuffer->CreateAndAddLookupNode(mLinkParentNames[i], mLinkNames[i]);
  }
}


void vtkMRMLROS2RobotNode::SetRobotName(const std::string & robotName)
{
  mROS2RobotName = robotName;
}

void vtkMRMLROS2RobotNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
}

void vtkMRMLROS2RobotNode::WriteXML( ostream& of, int nIndent )
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(ROS2RobotName, ROS2RobotName);
  vtkMRMLWriteXMLEndMacro();
}

void vtkMRMLROS2RobotNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(ROS2RobotName, ROS2RobotName);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}
