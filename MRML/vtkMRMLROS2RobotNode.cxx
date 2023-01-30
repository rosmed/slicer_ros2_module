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
#include <vtkMRMLTransformNode.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkPointSet.h>
#include <vtkMRMLModelNode.h>
#include <vtkMRMLModelDisplayNode.h>
#include <vtkSTLReader.h>
#include <regex>
#include <ament_index_cpp/get_package_share_directory.hpp>

auto const MM_TO_M_CONVERSION = 1000.00;


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
    vtkErrorMacro(<< "SetRobotDescriptionParameterNode, robot node needs to be added to the scene first");
    return false;
  }
  mRobotDescriptionParameterNode = param;
  ObserveParameterNode(param);
  return true;
}

void vtkMRMLROS2RobotNode::ObserveParameterNode(vtkMRMLROS2ParameterNode * node )
{
  // Set up the observer for the robot state publisher 
  if (!this->GetScene()->GetNodeByID(node->GetID())){
    vtkErrorMacro(<< "Robot node is not in the scene.");
    return;
  }
  node->AddObserver(vtkMRMLROS2ParameterNode::ParameterModifiedEvent, this, &vtkMRMLROS2RobotNode::ObserveParameterNodeCallback);
  this->SetAndObserveNodeReferenceID("ObservedParameter", node->GetID());
}

void vtkMRMLROS2RobotNode::ObserveParameterNodeCallback( vtkObject* caller, unsigned long, void* vtkNotUsed(callData))
{
  // Manage parameter callback when robot description is available
  vtkMRMLROS2ParameterNode* parameterNode = vtkMRMLROS2ParameterNode::SafeDownCast(caller);
  if (!parameterNode)
  {
    return;
  }
  else
  {
    mRobotDescription = mRobotDescriptionParameterNode->GetParameterAsString("robot_description");
    ParseRobotDescription();
  }
}

bool vtkMRMLROS2RobotNode::ParseRobotDescription()
{
  // Parser the urdf file into an urdf model - to get names of links and pos/ rpy
  if (!mInternals->mURDFModel.initString(mRobotDescription)) {
      return false;
  }
  return true;
}

void vtkMRMLROS2RobotNode::InitializeLookupListFromURDF()
{
  // This function goes through the urdf file and populates a list of the parents and children of
  // each link transform. This is later used to initialize the robots lookup nodes.

  // Start with the root (base of the robot)
  auto root = mInternals->mURDFModel.getRoot();
  std::string root_name = root->name;
  mLinkNames.push_back(root_name);
  mLinkParentNames.push_back(root_name);
  mInternals->mVisualVector.push_back(root->visual);

  // Go through the rest of the robot and save to list
  size_t lastExplored = 0;
  while (lastExplored != mInternals->mVisualVector.size()){
    mInternals->mParentLinkPointer = mInternals->mURDFModel.getLink(mLinkNames[lastExplored]);
    mInternals->mChildLinkPointer =  mInternals->mParentLinkPointer->child_links;

    for (auto i: mInternals->mChildLinkPointer) { 
      mLinkNames.push_back(i->name);
      mLinkParentNames.push_back(mInternals->mParentLinkPointer->name);
      mInternals->mVisualVector.push_back(i->visual); // need to get the origin from the visual
    }
    lastExplored++;
  }
  mLookups.resize(mLinkNames.size());
  mNumberOfLinks = mLinkNames.size();
}

void vtkMRMLROS2RobotNode::InitializeOffsetListAndModelFilesFromURDF()
{
  // This function goes through the urdf file to obtain the offset for each link and store it in a list.
  // We also get the filename for each stl model for visual loading later.

  // Resize the storage arrays
  mLinkModelFiles.resize(mInternals->mVisualVector.size());
  mInternals->mLinkOrigins.resize(mInternals->mVisualVector.size());

  // Get the origin and the file names
  for (size_t index = 0; index < mNumberOfLinks; ++index) {
    auto i = mInternals->mVisualVector[index];
    if (i == nullptr) {
      vtkErrorMacro(<< "No visual vector available for this link");
    } 
    else {
    //   urdf::Pose origin;
      auto origin = i->origin;
      mInternals->mLinkOrigins[index] = origin;
      // Get stl file name and add it to a list of vectors for python parsing later
      std::shared_ptr<urdf::Mesh> mesh =  std::dynamic_pointer_cast<urdf::Mesh>(i->geometry); // How do I put this in the internals??
      if (mesh != nullptr) {
        // See if the file name uses a package url
        std::string filename = mesh->filename;
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
      } else {
        vtkErrorMacro(<< "Link" << index << " has a visual, but not from a file");
      }
    }
  }
}

void vtkMRMLROS2RobotNode::InitializeLookups()
{
  // Initialize the lookups for the robot based on the previously stored parent and children names of the transform.
  for (size_t i = 0; i < mNumberOfLinks; i++){
    mROS2Node->mBuffer->CreateAndAddLookupNode(mLinkParentNames[i], mLinkNames[i]);
  }
}

void vtkMRMLROS2RobotNode::InitializeOffsets()
{
  // Initialize the offset transforms for each link
  for (size_t i = 0; i < mNumberOfLinks; i++){

    // Create the transform node
    vtkSmartPointer<vtkMRMLTransformNode> transformNode = vtkMRMLTransformNode::New();
    this->GetScene()->AddNode(transformNode);
    transformNode->SetName((mLinkNames[i] + "_offset").c_str());

    // Translate
    auto origin = mInternals->mLinkOrigins[i];
    vtkSmartPointer<vtkTransform> transform = vtkTransform::New();
    transform->Translate(origin.position.x*MM_TO_M_CONVERSION, origin.position.y*MM_TO_M_CONVERSION, origin.position.z*MM_TO_M_CONVERSION);
    transformNode->SetAndObserveTransformToParent(transform);

    // Rotate
    double r = 0.0;
    double p = 0.0;
    double y = 0.0;
    origin.rotation.getRPY(r, p, y);
    transform->RotateZ(y * (180.0/M_PI)); // RAD to degree conversion - use math.pi instead
    transform->RotateY(p * (180.0/M_PI));
    transform->RotateX(r * (180.0/M_PI));
    transform->Modified();

    // Scale
    vtkNew<vtkMatrix4x4> offsetMatrix;
    transformNode->GetMatrixTransformToParent(offsetMatrix);
    vtkNew<vtkMatrix4x4> MmToM_Transform;
    MmToM_Transform->SetElement(0, 0, MM_TO_M_CONVERSION);
    MmToM_Transform->SetElement(1, 1, MM_TO_M_CONVERSION);
    MmToM_Transform->SetElement(2, 2, MM_TO_M_CONVERSION);
    MmToM_Transform->Multiply4x4(offsetMatrix, MmToM_Transform, offsetMatrix);

    transformNode->SetAndObserveTransformToParent(transform);
    transformNode->SetMatrixTransformToParent(offsetMatrix);
    transform->Modified();
  } 
}

void vtkMRMLROS2RobotNode::LoadLinkModels()
{ 
  // Load the stl models for each link
  for (size_t i = 0; i < (mInternals->mVisualVector.size()); i++){
    std::string filename = mLinkModelFiles[i];
    vtkNew<vtkSTLReader> reader; // default is STL
    reader->SetFileName(mLinkModelFiles[i].c_str());
    reader->Update();

    vtkSmartPointer<vtkPointSet> meshFromFile;
    meshFromFile = reader->GetOutput();
    vtkSmartPointer<vtkPointSet> meshToSetInNode;

    vtkSmartPointer<vtkMRMLModelNode> modelNode = vtkMRMLModelNode::New();
    this->GetScene()->AddNode( modelNode.GetPointer() );
    modelNode->SetName((mLinkNames[i] + "_model").c_str());
    modelNode->SetAndObserveMesh(meshFromFile);
    
    // Create display node
    if (modelNode->GetDisplayNode() == NULL){
        vtkNew< vtkMRMLModelDisplayNode > displayNode;
        this->GetScene()->AddNode( displayNode.GetPointer() );
        displayNode->SetName((mLinkNames[i] + "_model_display_node").c_str());
        modelNode->SetAndObserveDisplayNodeID( displayNode->GetID() );
    }
  }
}


void vtkMRMLROS2RobotNode::SetupTransformTree(){
  
  // This function is used to setup the transform hierarchy to visualize the robot
  // The tree is cascaded lookups (which correspond to the transforms that come from tf2) and each 
  // lookup has an offset associated with it. This offset corresponds to the transformation between that 
  // link (the child of the lookup) to the base of the robot. The model for each link sits on (observes)
  // it's corresponding offset

  // Setup models on their corresponding offsets
  for (size_t i = 0; i < mNumberOfLinks; i++){
    vtkSmartPointer<vtkMRMLModelNode> linkModel = vtkMRMLModelNode::SafeDownCast(this->GetScene()->GetFirstNodeByName((mLinkNames[i] + "_model").c_str()));
    vtkSmartPointer<vtkMRMLTransformNode> transformNode = vtkMRMLTransformNode::SafeDownCast(this->GetScene()->GetFirstNodeByName((mLinkNames[i] + "_offset").c_str()));
    linkModel->SetAndObserveTransformNodeID(transformNode->GetID());
  }

  // Cascade the lookups
  for (size_t i = 0; i < mNumberOfLinks; i++){
    auto lookup = mROS2Node->mBuffer->mLookupNodes[i];
    std::string parent = lookup->GetParentID();
    for (size_t j = 0; j < mNumberOfLinks; j++){
      auto potentialParent = mROS2Node->mBuffer->mLookupNodes[j];
      std::string child = potentialParent->GetChildID();
      if (child == parent){
        lookup->SetAndObserveTransformNodeID(potentialParent->GetID());
      }
    }
  }

  // Setup offsets (and models) on their corresponding lookups
  for (size_t i = 0; i < (mNumberOfLinks); i++){
    auto lookupNode = mROS2Node->mBuffer->mLookupNodes[i];
    std::string linkName = lookupNode->GetChildID();
    std::string p = lookupNode->GetParentID();
    vtkSmartPointer<vtkMRMLTransformNode> offsetTransform = vtkMRMLTransformNode::SafeDownCast(this->GetScene()->GetFirstNodeByName((linkName + "_offset").c_str()));
    if(offsetTransform){
      offsetTransform->SetAndObserveTransformNodeID(lookupNode->GetID());
    }
  }

}

void vtkMRMLROS2RobotNode::SetupRobotVisualization(){
  // This function pulls all the pieces together
  // Initialize lookups and offsets, load models, setup the transform tree
  InitializeLookupListFromURDF();
  InitializeOffsetListAndModelFilesFromURDF();
  InitializeOffsets();
  InitializeLookups();
  LoadLinkModels();
  SetupTransformTree();
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
