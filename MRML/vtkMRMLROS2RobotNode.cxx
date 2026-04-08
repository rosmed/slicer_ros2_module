#include <vtkMRMLROS2RobotNode.h>
#include <vtkMRMLROS2RobotNodeInternals.h>

#include <vtkEventBroker.h>
#include <vtkTransform.h>
#include <vtkPointSet.h>
#include <vtkSTLReader.h>
#include <vtkOBJReader.h>
#include <vtkSphereSource.h>
#include <vtkAlgorithmOutput.h>
#include <vtkTransformFilter.h>
#include <vtksys/SystemTools.hxx>
#include "vtkAssImpConversion.h"

#include <vtkMRMLScene.h>
#include <vtkMRMLTransformNode.h>
#include <vtkMRMLLinearTransformNode.h>
#include <vtkMRMLModelNode.h>
#include <vtkMRMLModelDisplayNode.h>

#include <vtkMRMLROS2Utils.h>
#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2ParameterNode.h>
#include <vtkMRMLROS2Tf2LookupNode.h>

#include <regex>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vtkMRMLROS2NodeInternals.h>
#include <eigen3/Eigen/Geometry>
#include <sstream>
#include <unordered_map>
#include <algorithm>
#include <map>
#include <thread>
#include <queue>

#include <vtkMoveitMsgsRobotTrajectory.h>
#include <vtkROS2ToSlicer.h>
#include <vtkSlicerToROS2.h>

#include <QTimer>

// MoveIt kinematics and planning includes
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
// ROS2 parameter client for reading remote node parameters
#include <rclcpp/parameter_client.hpp>
#include <chrono>

// KDL includes
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

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
  this->RemoveRobotVisualization();
}


void vtkMRMLROS2RobotNode::RemoveRobotVisualization()
{
  if (!this->GetScene()) {
    return;
  }

  // Remove nodes referenced as "model"
  int nbModelRefs = this->GetNumberOfNodeReferences("model");
  for (int i = nbModelRefs - 1; i >= 0; --i) {
    vtkMRMLNode* node = this->GetNthNodeReference("model", i);
    if (node) {
      // 1. Remove the display node(s) associated with this model
      vtkMRMLModelNode* modelNode = vtkMRMLModelNode::SafeDownCast(node);
      if (modelNode) {
        modelNode->RemoveAllDisplayNodeIDs();
      }
      // 2. Remove the model node itself
      this->GetScene()->RemoveNode(node);
    }
    this->RemoveNthNodeReferenceID("model", i);
  }

  // Remove nodes referenced as "lookup" (and their children should handle themselves if properly observed)
  int nbLookupRefs = this->GetNumberOfNodeReferences("lookup");
  for (int i = nbLookupRefs - 1; i >= 0; --i) {
    vtkMRMLNode* node = this->GetNthNodeReference("lookup", i);
    if (node) {
      // 2. Remove the lookup node
      this->GetScene()->RemoveNode(node);
    }
    this->RemoveNthNodeReferenceID("lookup", i);
  }

  // Clear transient vectors in internals
  mInternals->mLinkNames.clear();
  mInternals->mLinkParentNames.clear();
  mInternals->mLinkModelFiles.clear();
  mInternals->mLinkModels.clear();
  mInternals->mLookupNodes.clear();

  mNumberOfLinks = 0;
}


bool vtkMRMLROS2RobotNode::AddToROS2Node(const char * nodeId,
                                         const std::string & robotName,
                                         const std::string & parameterNodeName,
                                         const std::string & parameterName,
                                         const std::string & fixedFrame,
                                         const std::string & tfPrefix)
{
  this->SetName(mMRMLNodeName.c_str());
  std::string errorMessage;
  vtkMRMLROS2NodeNode * mrmlROSNodePtr = vtkMRMLROS2::CheckROS2NodeExists(this, nodeId, errorMessage);
  if (!mrmlROSNodePtr) {
    vtkErrorMacro(<< "AddToROS2Node: " << errorMessage);
    return false;
  }
  // Add the robot to the ros2 node
  mrmlROSNodePtr->SetNthNodeReferenceID("robot",
                                        mrmlROSNodePtr->GetNumberOfNodeReferences("robot"),
                                        this->GetID());
  this->SetNodeReferenceID("node", nodeId);
  mMRMLROS2Node = mrmlROSNodePtr;
  mInternals->mParameterNodeName = parameterNodeName;
  mInternals->mParameterName = parameterName;
  mInternals->mFixedFrame = fixedFrame;
  if ((tfPrefix == "") || ((*(tfPrefix.crend()) == '/'))) {
    mInternals->mTfPrefix = tfPrefix;
  } else {
    mInternals->mTfPrefix = tfPrefix + '/';
  }
  SetRobotDescriptionParameterNode();
  SetRobotName(robotName);
  return true;
}


bool vtkMRMLROS2RobotNode::SetRobotDescriptionParameterNode(void)
{
  // Check if the node is in the scene
  if (!this->GetScene()) {
    vtkErrorMacro(<< "SetRobotDescriptionParameterNode: robot node needs to be added to the scene first");
    return false;
  }
  // Create a new parameter node
  mRobotDescriptionParameterNode = vtkMRMLROS2ParameterNode::New();
  this->GetScene()->AddNode(mRobotDescriptionParameterNode);
  mRobotDescriptionParameterNode->SetName((mRobotName + "_parameter").c_str());
  mRobotDescriptionParameterNode->AddToROS2Node(mMRMLROS2Node->GetID(), mInternals->mParameterNodeName);
  mRobotDescriptionParameterNode->AddParameter(mInternals->mParameterName);
  ObserveParameterNode(mRobotDescriptionParameterNode);
  return true;
}


void vtkMRMLROS2RobotNode::ObserveParameterNode(vtkMRMLROS2ParameterNode * node)
{
  // Set up the observer for the robot state publisher
  if (!this->GetScene()->GetNodeByID(node->GetID())) {
    vtkErrorMacro(<< "ObserveParameterNode: robot node is not in the scene.");
    return;
  }
  node->AddObserver(vtkMRMLROS2ParameterNode::ParameterModifiedEvent, this, &vtkMRMLROS2RobotNode::ObserveParameterNodeCallback);
  this->SetAndObserveNodeReferenceID("parameter", node->GetID());
}


void vtkMRMLROS2RobotNode::ObserveParameterNodeCallback( vtkObject* caller, unsigned long, void* vtkNotUsed(callData))
{
  // Manage parameter callback when robot description is available
  vtkMRMLROS2ParameterNode* parameterNode = vtkMRMLROS2ParameterNode::SafeDownCast(caller);
  if (!parameterNode) {
    vtkErrorMacro(<< "ObserveParameterNodeCallback: parameter node is not valid");
    return;
  }
  // Use IsParameterSet to check if the parameter is set
  if (!mRobotDescriptionParameterNode->IsParameterSet(mInternals->mParameterName)) {
    // Silently return until parameter is available
    return;
  }

  if (mRobotDescriptionParameterNode->GetParameterType(mInternals->mParameterName) != "string" &&
      mRobotDescriptionParameterNode->GetParameterType(mInternals->mParameterName) != "String") {
    std::string outtype = mRobotDescriptionParameterNode->GetParameterType(mInternals->mParameterName);
    vtkErrorMacro(<< "ObserveParameterNodeCallback: parameter \"" << mInternals->mParameterName << "\" is of type " << outtype << " and not string.");
    return;
  }

  std::string newDescription = mRobotDescriptionParameterNode->GetParameterAsString(mInternals->mParameterName);
  if (newDescription != mInternals->mRobotDescription || mNumberOfLinks == 0) {
    mInternals->mRobotDescription = newDescription;
    if (ParseRobotDescription()) {
      SetupRobotVisualization();
    } else {
      vtkErrorMacro(<< "ObserveParameterNodeCallback: Failed to parse robot description");
    }
  }

}


bool vtkMRMLROS2RobotNode::RemoveGoalStateRobot()
{
  vtkMRMLScene* scene = this->GetScene();
  if (!scene) {
    vtkErrorMacro(<< "RemoveGoalStateRobot: robot node needs to be added to the scene first");
    return false;
  }

  auto removeGoalReferences = [&](const char* role) {
    int count = this->GetNumberOfNodeReferences(role);
    for (int i = count - 1; i >= 0; --i) {
      vtkMRMLNode* node = this->GetNthNodeReference(role, i);
      if (node) {
        scene->RemoveNode(node);
      }
    }
  };

  removeGoalReferences("goal_model");
  removeGoalReferences("goal_transform");
  return true;
}


bool vtkMRMLROS2RobotNode::CreateGoalStateRobot(vtkMRMLROS2RobotNode * sourceRobot)
{
  vtkMRMLScene* scene = this->GetScene();
  if (!scene) {
    vtkErrorMacro(<< "CreateGoalStateRobot: robot node needs to be added to the scene first");
    return false;
  }

  if (!sourceRobot) {
    vtkErrorMacro(<< "CreateGoalStateRobot: source robot is null");
    return false;
  }

  if (!sourceRobot->GetScene()) {
    vtkErrorMacro(<< "CreateGoalStateRobot: source robot must be added to the scene first");
    return false;
  }

  RemoveGoalStateRobot();

  int modelCount = sourceRobot->GetNumberOfNodeReferences("model");
  std::cout << "Found " << modelCount << " model nodes to duplicate" << std::endl;

  std::vector<vtkSmartPointer<vtkMRMLLinearTransformNode>> goalTransforms;
  goalTransforms.reserve(modelCount);

  for (int i = 0; i < modelCount; ++i) {
    vtkMRMLModelNode* original = vtkMRMLModelNode::SafeDownCast(sourceRobot->GetNthNodeReference("model", i));
    if (!original) {
      continue;
    }

    vtkSmartPointer<vtkMRMLLinearTransformNode> goalTransform = vtkSmartPointer<vtkMRMLLinearTransformNode>::New();
    scene->AddNode(goalTransform);
    std::string transformName = std::string(original->GetName() ? original->GetName() : "model") + "_goal_transform";
    goalTransform->SetName(transformName.c_str());

    vtkMRMLLinearTransformNode* origTransform = vtkMRMLLinearTransformNode::SafeDownCast(
      scene->GetNodeByID(original->GetTransformNodeID()));
    if (origTransform) {
      vtkNew<vtkMatrix4x4> matrix;
      origTransform->GetMatrixTransformToParent(matrix);
      goalTransform->SetMatrixTransformToParent(matrix);
    }
    goalTransforms.push_back(goalTransform);

    vtkSmartPointer<vtkMRMLModelNode> goal = vtkSmartPointer<vtkMRMLModelNode>::New();
    scene->AddNode(goal);

    std::string goalName = std::string(original->GetName() ? original->GetName() : "model") + "_goal";
    goal->SetName(goalName.c_str());
    std::cout << "  Creating goal: " << goalName << " with transform: " << transformName << std::endl;

    if (original->GetMesh()) {
      goal->SetAndObserveMesh(original->GetMesh());
    }

    vtkMRMLModelDisplayNode* origDisp = vtkMRMLModelDisplayNode::SafeDownCast(original->GetDisplayNode());
    vtkNew<vtkMRMLModelDisplayNode> goalDisp;
    scene->AddNode(goalDisp.GetPointer());
    if (origDisp) {
      goalDisp->Copy(origDisp);
    }
    goalDisp->SetColor(0.0, 1.0, 1.0);
    goalDisp->SetOpacity(0.30);
    goal->SetAndObserveDisplayNodeID(goalDisp->GetID());

    goal->SetAndObserveTransformNodeID(goalTransform->GetID());
    this->AddNodeReferenceID("goal_model", goal->GetID());
    this->AddNodeReferenceID("goal_transform", goalTransform->GetID());
  }

  int lookupCount = sourceRobot->GetNumberOfNodeReferences("lookup");
  for (int i = 0; i < lookupCount; ++i) {
    vtkMRMLROS2Tf2LookupNode* lookup = vtkMRMLROS2Tf2LookupNode::SafeDownCast(sourceRobot->GetNthNodeReference("lookup", i));
    if (!lookup) {
      continue;
    }
    std::string parentFrame = lookup->GetParentID();
    for (int j = 0; j < lookupCount; ++j) {
      vtkMRMLROS2Tf2LookupNode* potentialParent = vtkMRMLROS2Tf2LookupNode::SafeDownCast(sourceRobot->GetNthNodeReference("lookup", j));
      if (!potentialParent) {
        continue;
      }
      std::string childFrame = potentialParent->GetChildID();
      if (childFrame == parentFrame && i != j) {
        if (i < static_cast<int>(goalTransforms.size()) && j < static_cast<int>(goalTransforms.size())) {
          goalTransforms[i]->SetAndObserveTransformNodeID(goalTransforms[j]->GetID());
        }
        break;
      }
    }
  }

  for (int i = 0; i < lookupCount && i < static_cast<int>(goalTransforms.size()); ++i) {
    vtkMRMLROS2Tf2LookupNode* lookup = vtkMRMLROS2Tf2LookupNode::SafeDownCast(sourceRobot->GetNthNodeReference("lookup", i));
    if (!lookup) {
      continue;
    }
    vtkNew<vtkMatrix4x4> matrix;
    lookup->GetMatrixTransformToParent(matrix);
    goalTransforms[i]->SetMatrixTransformToParent(matrix);
    goalTransforms[i]->Modified();
  }

  vtkSmartPointer<vtkMRMLROS2RobotNode> sourceRobotSafe = sourceRobot;
  QTimer::singleShot(400, [this, sourceRobotSafe, goalTransforms]() {
    int lc = sourceRobotSafe->GetNumberOfNodeReferences("lookup");
    for (int i = 0; i < lc && i < static_cast<int>(goalTransforms.size()); ++i) {
      vtkMRMLROS2Tf2LookupNode* lookup = vtkMRMLROS2Tf2LookupNode::SafeDownCast(sourceRobotSafe->GetNthNodeReference("lookup", i));
      if (!lookup) {
        continue;
      }
      vtkNew<vtkMatrix4x4> matrix;
      lookup->GetMatrixTransformToParent(matrix);
      goalTransforms[i]->SetMatrixTransformToParent(matrix);
      goalTransforms[i]->Modified();
    }
  });

  std::cout << "goal creation complete!" << std::endl;
  return true;
}


bool vtkMRMLROS2RobotNode::ParseRobotDescription(void)
{
  // Parser the urdf file into an urdf model - to get names of links and pos/ rpy
  if (!mInternals->mURDFModel.initString(mInternals->mRobotDescription)) {
    vtkErrorMacro(<< "ParseRobotDescription: failed to parse robot description");
    return false;
  }
  return true;
}


void vtkMRMLROS2RobotNode::SetupRobotVisualization(void)
{
  // Before setting up new visualization, remove any existing robot nodes 
  // to avoid duplication and leaks.
  this->RemoveRobotVisualization();

  // 1. Initialize lookup list and visual vectors from URDF
  auto root = mInternals->mURDFModel.getRoot();
  if (!root) {
    vtkErrorMacro(<< "SetupRobotVisualization: root link not found in URDF model");
    return;
  }
  
  std::string root_name = root->name;
  mInternals->mLinkNames.push_back(root_name);
  mInternals->mLinkParentNames.push_back(root_name);
  mInternals->mVisualVector.push_back(root->visual);
  mInternals->mMaterialsMap = mInternals->mURDFModel.materials_;
  mInternals->mLinkMaterials.push_back(root->visual != nullptr ? root->visual->material_name : "");
  mInternals->mLinkOrigins.push_back(root->visual != nullptr ? root->visual->origin : urdf::Pose());

  // BFS to explore links
  size_t lastExplored = 0;
  while (lastExplored < mInternals->mVisualVector.size()) {
    auto parentLink = mInternals->mURDFModel.getLink(mInternals->mLinkNames[lastExplored]);
    if (parentLink) {
      for (const auto& childLink : parentLink->child_links) {
        if (!childLink) continue;
        mInternals->mLinkNames.push_back(childLink->name);
        mInternals->mLinkParentNames.push_back(parentLink->name);
        mInternals->mVisualVector.push_back(childLink->visual);
        mInternals->mLinkMaterials.push_back(childLink->visual != nullptr ? childLink->visual->material_name : "");
        mInternals->mLinkOrigins.push_back(childLink->visual != nullptr ? childLink->visual->origin : urdf::Pose());
      }
    }
    lastExplored++;
  }
  mNumberOfLinks = mInternals->mLinkNames.size();

  // 2. Resolve mesh filenames
  mInternals->mLinkModelFiles.resize(mNumberOfLinks);
  for (size_t index = 0; index < mNumberOfLinks; ++index) {
    auto visual = mInternals->mVisualVector[index];
    if (visual && visual->geometry) {
      auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(visual->geometry);
      if (mesh) {
        std::string filename = mesh->filename;
        std::regex pkg_regex("^package:\\/\\/([a-zA-Z0-9_-]+)\\/(.*)");
        std::smatch match;
        if (std::regex_search(filename, match, pkg_regex)) {
          try {
            filename = ament_index_cpp::get_package_share_directory(match[1]) + "/" + std::string(match[2]);
          } catch (...) {
            vtkErrorMacro(<< "Failed to resolve package " << std::string(match[1]));
          }
        }
        mInternals->mLinkModelFiles[index] = filename;
      }
    }
  }

  // 3. Create MRML nodes (Lookups and Models)
  for (size_t i = 0; i < mNumberOfLinks; i++) {
    // Create Lookup
    vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookup;
    std::string parentName = mInternals->mLinkParentNames[i];
    std::string childName = mInternals->mLinkNames[i];
    
    // For fixed frames, use the provided name, otherwise use prefix
    if (i == 0 && !mInternals->mFixedFrame.empty()) {
      lookup = mMRMLROS2Node->CreateAndAddTf2LookupNode(mInternals->mFixedFrame, mInternals->mTfPrefix + childName);
    } else {
      lookup = mMRMLROS2Node->CreateAndAddTf2LookupNode(mInternals->mTfPrefix + parentName, mInternals->mTfPrefix + childName);
    }
    mInternals->mLookupNodes.push_back(lookup);
    this->SetNthNodeReferenceID("lookup", i, lookup ? lookup->GetID() : nullptr);

    // Create Model Node and apply local offset
    auto visual = mInternals->mVisualVector[i];
    vtkSmartPointer<vtkPointSet> meshData;
    if (visual) {
      meshData = LoadModelFile(mInternals->mLinkModelFiles[i]);
      
      // Apply URDF scale and unit conversion
      double sx = 1.0, sy = 1.0, sz = 1.0;
      auto meshGeom = std::dynamic_pointer_cast<urdf::Mesh>(visual->geometry);
      if (meshGeom) { sx = meshGeom->scale.x; sy = meshGeom->scale.y; sz = meshGeom->scale.z; }
      
      vtkNew<vtkTransform> scaleTransform;
      scaleTransform->Scale(sx * 1000.0, sy * 1000.0, sz * 1000.0);
      
      vtkNew<vtkTransformFilter> tf;
      tf->SetInputData(meshData);
      tf->SetTransform(scaleTransform);
      tf->Update();
      meshData = tf->GetOutput();
    }

    vtkNew<vtkMRMLModelNode> modelNode;
    this->GetScene()->AddNode(modelNode);
    modelNode->SetName((mInternals->mLinkNames[i] + "_model").c_str());
    modelNode->SetAndObserveMesh(meshData);
    mInternals->mLinkModels.push_back(modelNode);
    this->SetNthNodeReferenceID("model", i, modelNode->GetID());

    // Local offset transform
    vtkNew<vtkTransform> offsetTf;
    auto origin = mInternals->mLinkOrigins[i];
    offsetTf->Translate(origin.position.x * 1000.0, origin.position.y * 1000.0, origin.position.z * 1000.0);
    double r, p, y;
    origin.rotation.getRPY(r, p, y);
    offsetTf->RotateZ(y * (180.0/M_PI));
    offsetTf->RotateY(p * (180.0/M_PI));
    offsetTf->RotateX(r * (180.0/M_PI));
    modelNode->ApplyTransform(offsetTf);

    // Display Node
    vtkNew<vtkMRMLModelDisplayNode> displayNode;
    this->GetScene()->AddNode(displayNode);
    modelNode->SetAndObserveDisplayNodeID(displayNode->GetID());
    if (!mInternals->mMaterialsMap.empty() && !mInternals->mLinkMaterials[i].empty()) {
      auto mat = mInternals->mMaterialsMap[mInternals->mLinkMaterials[i]];
      if (mat) displayNode->SetColor(mat->color.r, mat->color.g, mat->color.b);
    } else {
      displayNode->SetColor(0.5, 0.5, 0.5);
    }
  }

  // 4. Finalize Transform Tree
  for (size_t i = 0; i < mInternals->mLookupNodes.size(); i++) {
    auto lookup = mInternals->mLookupNodes[i];
    if (!lookup) continue;
    lookup->SetModifiedOnLookup(i == 0);
    std::string parent = lookup->GetParentID();
    for (size_t j = 0; j < mInternals->mLookupNodes.size(); j++) {
      auto potentialParent = mInternals->mLookupNodes[j];
      if (potentialParent && potentialParent->GetChildID() == parent && i != j) {
        lookup->SetAndObserveTransformNodeID(potentialParent->GetID());
      }
    }
    if (mInternals->mLinkModels[i]) {
      mInternals->mLinkModels[i]->SetAndObserveTransformNodeID(lookup->GetID());
    }
  }

  SetupKDLIKWithLimits();

  // Clear transient vectors
  mInternals->mLinkModels.clear();
  mInternals->mLookupNodes.clear();
  mInternals->mLinkNames.clear();
  mInternals->mLinkParentNames.clear();
  mInternals->mLinkModelFiles.clear();
}

vtkSmartPointer<vtkPointSet> vtkMRMLROS2RobotNode::LoadModelFile(const std::string& filename)
{
  if (filename.empty()) return nullptr;
  
  std::string ext = vtksys::SystemTools::LowerCase(vtksys::SystemTools::GetFilenameLastExtension(filename));
  vtkSmartPointer<vtkAlgorithmOutput> port;

  if (ext == ".stl") {
    vtkNew<vtkSTLReader> reader;
    reader->SetFileName(filename.c_str());
    reader->Update();
    return reader->GetOutput();
  } else if (ext == ".obj") {
    vtkNew<vtkOBJReader> reader;
    reader->SetFileName(filename.c_str());
    reader->Update();
    return reader->GetOutput();
  } else {
    vtkSmartPointer<vtkPolyData> polyData = vtkAssImpConversion::vtkAssImpToPolyData(filename);
    if (polyData) return polyData;
  }
  
  vtkErrorMacro(<< "Failed to load model file: " << filename << ". Using fallback sphere.");
  vtkNew<vtkSphereSource> sphere;
  sphere->SetRadius(10.0);
  sphere->Update();
  return sphere->GetOutput();
}


void vtkMRMLROS2RobotNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
}


void vtkMRMLROS2RobotNode::WriteXML(ostream& of, int nIndent)
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(RobotName, RobotName);
  vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2RobotNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(RobotName, RobotName);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}

// MoveIt IK implementation (commented out for faster build)
bool vtkMRMLROS2RobotNode::setupIKmoveit(const std::string & groupName)
{

  if (!mMRMLROS2Node) {
    vtkErrorMacro(<< "setupIK: ROS2 node not available");
    return false;
  }

  if (mInternals->mRobotDescription.empty()) {
    vtkErrorMacro(<< "setupIK: robot description not available");
    return false;
  }

  try {
    auto node = mMRMLROS2Node->mInternals->mNodePointer;
    std::string prefix = "robot_description_kinematics." + groupName;

    // Helper for declaring/updating parameters
    auto ensureParam = [&](const std::string& name, auto value) {
      if (!node->has_parameter(name)) {
        node->declare_parameter(name, value);
      } else {
        node->set_parameter(rclcpp::Parameter(name, value));
      }
    };

    // Set kinematics parameters
    ensureParam(prefix + ".kinematics_solver", std::string("kdl_kinematics_plugin/KDLKinematicsPlugin"));
    ensureParam(prefix + ".kinematics_solver_search_resolution", 0.005);
    ensureParam(prefix + ".kinematics_solver_timeout", 0.05);

    // Load and cache RobotModel
    mInternals->RobotModelLoaderPtr = std::make_unique<robot_model_loader::RobotModelLoader>(node, "robot_description");
    mInternals->RobotModelPtr = mInternals->RobotModelLoaderPtr->getModel();

    if (!mInternals->RobotModelPtr) {
      vtkErrorMacro(<< "setupIK: Failed to load RobotModel");
      return false;
    }

    // Cache JointModelGroup
    mInternals->JointModelGroupPtr = mInternals->RobotModelPtr->getJointModelGroup(groupName);

    if (!mInternals->JointModelGroupPtr) {
      vtkErrorMacro(<< "setupIK: joint model group '" << groupName << "' not found");
      return false;
    }

    // Verify solver is available
    const auto& solver = mInternals->JointModelGroupPtr->getSolverInstance();

    if (!solver) {
      vtkErrorMacro(<< "setupIK: no kinematics solver for group '" << groupName << "'");
      return false;
    }

    mInternals->IKGroupName = groupName;
    return true;
  }
  catch (const std::exception& e) {
    vtkErrorMacro(<< "setupIK: exception - " << e.what());
    return false;
  }
}

std::string vtkMRMLROS2RobotNode::FindIKmoveit(vtkMatrix4x4* targetPose, const std::string& tipLink, const std::vector<double>& seedJointValues, double timeout)
{
  if (!targetPose) {
    vtkErrorMacro(<< "FindIK: target pose is null");
    return "";
  }

  // Setup IK if needed (only once, since we auto-discover the group)
  if (!mInternals->RobotModelPtr || !mInternals->JointModelGroupPtr) {
      vtkErrorMacro(<< "FindIK: setupIKmoveit failed");
      return "";
  }

  try {
    // Create robot state for solving
    moveit::core::RobotState robot_state(mInternals->RobotModelPtr);

    if (!seedJointValues.empty()) {
      robot_state.setJointGroupPositions(mInternals->JointModelGroupPtr, seedJointValues);
    } else {
      robot_state.setToDefaultValues();
    }

    vtkNew<vtkMatrix4x4> targetPoseSI;
    targetPoseSI->DeepCopy(targetPose);
    vtkMRMLROS2::ToSI(targetPoseSI);

    // Convert vtkMatrix4x4 to geometry_msgs Pose
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = targetPoseSI->GetElement(0, 3);  // mm to m
    pose_msg.position.y = targetPoseSI->GetElement(1, 3);
    pose_msg.position.z = targetPoseSI->GetElement(2, 3);

    // Extract rotation matrix and convert to quaternion
    Eigen::Matrix3d rot_matrix;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        rot_matrix(i, j) = targetPoseSI->GetElement(i, j);
      }
    }
    
    Eigen::Quaterniond quat(rot_matrix);
    pose_msg.orientation.x = quat.x();
    pose_msg.orientation.y = quat.y();
    pose_msg.orientation.z = quat.z();
    pose_msg.orientation.w = quat.w();

    // Call IK using setFromIK
    bool found_ik = robot_state.setFromIK(mInternals->JointModelGroupPtr, pose_msg, tipLink, timeout);
    if (!found_ik) {
      vtkWarningMacro(<< "FindIK: IK solution not found for group '" << mInternals->IKGroupName << "'");
    }

    // Extract joint values (or create NaN values if no solution found)
    std::vector<double> solution;
    robot_state.copyJointGroupPositions(mInternals->JointModelGroupPtr, solution);

    // Convert to comma-separated string
    std::ostringstream oss;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i > 0) oss << ",";
      oss << solution[i];
    }

    return oss.str();
  }
  catch (const std::exception& e) {
    vtkErrorMacro(<< "FindIK: exception - " << e.what());
    return "";
  }
}


bool vtkMRMLROS2RobotNode::SetupKDLIKWithLimits(void)
{
  try {
    auto rootAndTip = FindRootAndTipLinks();
    if (rootAndTip.size() < 2 || rootAndTip[0].empty() || rootAndTip[1].empty()) {
      vtkErrorMacro(<< "setupKDLIKWithLimits: Failed to determine root/tip links from URDF");
      return false;
    }
    const std::string& defaultRoot = rootAndTip[0];
    const std::string& defaultTip = rootAndTip[1];
    vtkInfoMacro(<< "Auto KDL setup with limits. Root: '" << defaultRoot
                 << "' Tip: '" << defaultTip << "'");

    // Create kdltree using KDL parser
    KDL::Tree kdlTree;
    if (!kdl_parser::treeFromString(mInternals->mRobotDescription, kdlTree)) {
      vtkErrorMacro(<< "setupKDLIKWithLimits: Failed to parse URDF to KDL tree");
      return false;
    }

    // Extract chain from tree
    mInternals->KDLChain = std::make_unique<KDL::Chain>();
    if (!kdlTree.getChain(defaultRoot, defaultTip, *mInternals->KDLChain)) {
      vtkErrorMacro(<< "setupKDLIKWithLimits: Failed to extract chain from " << defaultRoot << " to " << defaultTip);
      return false;
    }else{
      vtkInfoMacro(<< "setupKDLIKWithLimits: Successfully extracted chain from " << defaultRoot << " to " << defaultTip);
    }

    // Get number of joints in KDL chain
    unsigned int nj = mInternals->KDLChain->getNrOfJoints();

    // Initialize joint limits arrays
    mInternals->KDLJointMin = KDL::JntArray(nj);
    mInternals->KDLJointMax = KDL::JntArray(nj);

    // Extract joint limits from mURDFModel
    unsigned int joint_idx = 0;
    for (unsigned int i = 0; i < mInternals->KDLChain->getNrOfSegments(); i++) {
      const KDL::Segment& segment = mInternals->KDLChain->getSegment(i);
      const KDL::Joint& joint = segment.getJoint();
      
      if (joint.getType() != KDL::Joint::None) {
        std::string joint_name = joint.getName();
        // Print joint name for debugging
        vtkInfoMacro(<< "Processing joint: " << joint_name);
        auto urdf_joint = mInternals->mURDFModel.getJoint(joint_name);
          
          // Check if joint is continuous
          if (urdf_joint && urdf_joint->type == urdf::Joint::CONTINUOUS) {
              mInternals->KDLJointMin(joint_idx) = -2 * M_PI; 
              mInternals->KDLJointMax(joint_idx) = 2 * M_PI;
              vtkInfoMacro(<< "  Joint " << joint_name << " is CONTINUOUS. Setting wide limits [-2pi, 2pi].");
          } 
          else if (urdf_joint && urdf_joint->limits) {
            // Otherwise, get limits if available
            mInternals->KDLJointMin(joint_idx) = urdf_joint->limits->lower;
            mInternals->KDLJointMax(joint_idx) = urdf_joint->limits->upper;
          } 
          else {
            // Default limits if not specified
            mInternals->KDLJointMin(joint_idx) = -M_PI;
            mInternals->KDLJointMax(joint_idx) = M_PI;
          }
          joint_idx++;
        }
      }

    // Create solvers (NR_JL with joint limits)
    mInternals->KDLFkSolver = std::make_unique<KDL::ChainFkSolverPos_recursive>(*mInternals->KDLChain);
    mInternals->KDLIkSolverVel = std::make_unique<KDL::ChainIkSolverVel_pinv>(*mInternals->KDLChain);
    mInternals->KDLIkSolverJL = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
        *mInternals->KDLChain, mInternals->KDLJointMin, mInternals->KDLJointMax, *mInternals->KDLFkSolver, *mInternals->KDLIkSolverVel, 100, 1e-6);

    mInternals->KDLRootLink = defaultRoot;
    mInternals->KDLTipLink = defaultTip;
    mInternals->KDLUseJointLimits = true;

    // Print joint limits for verification
    vtkInfoMacro(<< "Joint limits for KDL IK solver:");
    for (unsigned int j = 0; j < nj; j++) {
      vtkInfoMacro(<< "  Joint " << j << ": [" << mInternals->KDLJointMin(j) << ", " << mInternals->KDLJointMax(j) << "]");
    }

    vtkInfoMacro(<< "setupKDLIKWithLimits: Successfully initialized KDL IK solver (NR_JL) for chain " 
                 << defaultRoot << " -> " << defaultTip 
                 << " with " << mInternals->KDLChain->getNrOfJoints() << " joints and joint limits");
 
    return true;
  }
  catch (const std::exception& e) {
    vtkErrorMacro(<< "setupKDLIKWithLimits: exception - " << e.what());
    return false;
  }
}

std::vector<std::string> vtkMRMLROS2RobotNode::FindRootAndTipLinks() const
{
  auto rootLink = mInternals->mURDFModel.getRoot();
  if (!rootLink) {
    vtkErrorMacro(<< "FindRootAndTipLinks: URDF root link is null");
    return {};
  }

  std::string rootLinkName = rootLink->name;
  std::string tipLinkName = rootLinkName;

  std::queue<std::shared_ptr<const urdf::Link>> bfsQueue;
  bfsQueue.push(rootLink);

  while (!bfsQueue.empty()) {
    auto currentLink = bfsQueue.front();
    bfsQueue.pop();

    if (!currentLink) {
      continue;
    }

    if (currentLink->child_links.empty()) {
      tipLinkName = currentLink->name;
      break;
    }

    for (const auto& childLink : currentLink->child_links) {
      bfsQueue.push(childLink);
    }
  }

  return {rootLinkName, tipLinkName};
}


std::string vtkMRMLROS2RobotNode::FindKDLIK(vtkMatrix4x4* targetPose, 
                                             const std::vector<double>& seedJointValues)
{
  if (!targetPose) {
    vtkErrorMacro(<< "FindKDLIK: target pose is null");
    return "";
  }

  if (!mInternals->KDLUseJointLimits && !mInternals->KDLIkSolver) {
    vtkErrorMacro(<< "FindKDLIK: KDL IK solver not initialized. Call setupKDLIK or setupKDLIKWithLimits first");
    return "";
  }

  if (mInternals->KDLUseJointLimits && !mInternals->KDLIkSolverJL) {
    vtkErrorMacro(<< "FindKDLIK: KDL IK solver with joint limits not initialized. Call setupKDLIKWithLimits first");
    return "";
  }

  try {
    // Convert vtkMatrix4x4 to KDL Frame
    vtkNew<vtkMatrix4x4> targetPoseSI;
    targetPoseSI->DeepCopy(targetPose);
    vtkMRMLROS2::ToSI(targetPoseSI);

    KDL::Frame targetFrame;
    
    // Set position (convert mm to m)
    targetFrame.p.x(targetPoseSI->GetElement(0, 3));
    targetFrame.p.y(targetPoseSI->GetElement(1, 3));
    targetFrame.p.z(targetPoseSI->GetElement(2, 3));
    
    // Set rotation
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        targetFrame.M(i, j) = targetPoseSI->GetElement(i, j);
      }
    }

    // Setup seed configuration
    KDL::JntArray qSeed(mInternals->KDLChain->getNrOfJoints());
    if (!seedJointValues.empty() && seedJointValues.size() == mInternals->KDLChain->getNrOfJoints()) {
      for (size_t i = 0; i < seedJointValues.size(); i++) {
        qSeed(i) = seedJointValues[i];
        vtkInfoMacro(<< "Seed joint " << i << ": " << qSeed(i));
      }
    } else {
      // Use zeros as default seed
      vtkInfoMacro(<< "FindKDLIK: Using zero joint angles as seed");
      qSeed.data.setZero();
    }

    // Solve IK
    KDL::JntArray qSolution(mInternals->KDLChain->getNrOfJoints());
    int result;
    
    if (mInternals->KDLUseJointLimits) {
      result = mInternals->KDLIkSolverJL->CartToJnt(qSeed, targetFrame, qSolution);
    } else {
      result = mInternals->KDLIkSolver->CartToJnt(qSeed, targetFrame, qSolution);
    }

    if (result < 0) {
      vtkWarningMacro(<< "FindKDLIK: IK solution not found (error code: " << result << ")");
      return "";
    }

    // Convert solution to comma-separated string
    std::ostringstream oss;
    for (unsigned int i = 0; i < qSolution.rows(); i++) {
      if (i > 0) oss << ",";
      oss << qSolution(i);
    }

    return oss.str();
  }
  catch (const std::exception& e) {
    vtkErrorMacro(<< "FindKDLIK: exception - " << e.what());
    return "";
  }
}


std::vector<std::string> vtkMRMLROS2RobotNode::GetSegments()
{
  std::vector<std::string> segmentNames;
  if (!mInternals->KDLChain) {
    vtkWarningMacro(<< "GetSegments: KDL chain not initialized");
    return segmentNames;
  }
  for (unsigned int i = 0; i < mInternals->KDLChain->getNrOfSegments(); i++) {
    const KDL::Segment& segment = mInternals->KDLChain->getSegment(i);
    segmentNames.push_back(segment.getName());
  }
  return segmentNames;
}

std::vector<std::string> vtkMRMLROS2RobotNode::GetJoints()
{
  std::vector<std::string> jointNames;
  if (!mInternals->KDLChain) {
    vtkWarningMacro(<< "GetJoints: KDL chain not initialized");
    return jointNames;
  }
  for (unsigned int i = 0; i < mInternals->KDLChain->getNrOfSegments(); i++) {
    const KDL::Segment& segment = mInternals->KDLChain->getSegment(i);
    const KDL::Joint& joint = segment.getJoint();
    if (joint.getType() != KDL::Joint::None) {
      jointNames.push_back(joint.getName());
    }
  }
  return jointNames;
}

vtkMatrix4x4* vtkMRMLROS2RobotNode::ComputeLocalTransform(const std::vector<double>& jointValues, vtkMatrix4x4* outTransform, const std::string& linkName)
{
  if (!outTransform || !mInternals->KDLChain) return nullptr;

  // 1. Find the Segment and its Joint Index
  size_t segmentIndex = 0;
  bool found = false;
  size_t kdlJointIndex = 0; // Tracks which "q" index corresponds to this segment

  for (unsigned int i = 0; i < mInternals->KDLChain->getNrOfSegments(); i++) {
    const KDL::Segment& seg = mInternals->KDLChain->getSegment(i);
    
    // Count moving joints up to this point to find the correct 'q' index
    if (seg.getJoint().getType() != KDL::Joint::None) {
        if (seg.getName() == linkName) {
            segmentIndex = i;
            found = true;
            break;
        }
        kdlJointIndex++;
    } else if (seg.getName() == linkName) {
        // Found it, but it's a fixed joint (no q index increment)
        segmentIndex = i;
        found = true;
        break;
    }
  }

  if (!found) {
     vtkErrorMacro(<< "Link '" << linkName << "' not found.");
     return nullptr;
  }

  // 2. Get the specific joint angle for this segment
  double q_val = 0.0;
  const KDL::Segment& targetSeg = mInternals->KDLChain->getSegment(segmentIndex);
  
  if (targetSeg.getJoint().getType() != KDL::Joint::None) {
      if (kdlJointIndex < jointValues.size()) {
          q_val = jointValues[kdlJointIndex];
      } else {
          vtkErrorMacro(<< "Joint index out of bounds.");
          return nullptr;
      }
  }

  // 3. Compute LOCAL Pose (Parent -> Child)
  // This is what Slicer needs for nested hierarchies.
  KDL::Frame localFrame = targetSeg.pose(q_val);

  // 4. Convert to VTK Matrix
  outTransform->Identity();
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      outTransform->SetElement(r, c, localFrame.M(r, c));
    }
    // Set translation (meters)
    outTransform->SetElement(r, 3, localFrame.p(r));
  }
  // Scale Meters -> Millimeters
  vtkMRMLROS2::FromSI(outTransform);

  return outTransform;
}

vtkMatrix4x4* vtkMRMLROS2RobotNode::ComputeKDLFK(const std::vector<double>& jointValues, vtkMatrix4x4* outTransform, const std::string& linkName)
{
  if (!outTransform) {
    vtkErrorMacro(<< "ComputeKDLFK: output transform is null");
    return nullptr;
  }
  if (!mInternals->KDLChain || !mInternals->KDLFkSolver) {
    vtkWarningMacro(<< "ComputeKDLFK: KDL chain or FK solver not initialized");
    return nullptr;
  }
  if (jointValues.size() != mInternals->KDLChain->getNrOfJoints()) {
    vtkErrorMacro(<< "ComputeKDLFK: expected " << mInternals->KDLChain->getNrOfJoints()
                  << " joint values but got " << jointValues.size());
    return nullptr;
  }

  unsigned int segmentIndex = mInternals->KDLChain->getNrOfSegments() - 1; 

  if (!linkName.empty()) {
    bool found = false;
    for (unsigned int i = 0; i < mInternals->KDLChain->getNrOfSegments(); i++) {
      if (mInternals->KDLChain->getSegment(i).getName() == linkName) {
        segmentIndex = i;
        found = true;
        break;
      }
    }
    if (!found) {
      vtkErrorMacro(<< "ComputeKDLFK: link '" << linkName << "' not found in KDL chain");
      return nullptr;
    }
  }

  KDL::JntArray q(mInternals->KDLChain->getNrOfJoints());
  for (unsigned int i = 0; i < q.rows(); i++) {
    q(i) = jointValues[i];
  }


  KDL::Frame frame;
  // KDL uses 1-based indexing for 'segmentNr', so we add 1 to our 0-based index.
  int result = mInternals->KDLFkSolver->JntToCart(q, frame, segmentIndex + 1);
  
  if (result < 0) {
    vtkErrorMacro(<< "ComputeKDLFK: KDL FK failed with error code " << result);
    return nullptr;
  }

  

  outTransform->Identity();

  // Copy Rotation
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      outTransform->SetElement(r, c, frame.M(r, c));
    }
  }

  // Copy Translation (Applying scale)
  outTransform->SetElement(0, 3, frame.p.x());
  outTransform->SetElement(1, 3, frame.p.y());
  outTransform->SetElement(2, 3, frame.p.z());
  vtkMRMLROS2::FromSI(outTransform);

  return outTransform;
}

vtkMoveitMsgsRobotTrajectory* vtkMRMLROS2RobotNode::PlanMoveItTrajectory(const std::string& groupName,
                                                         const std::vector<double>& goalJointValues,
                                                         double velocityScaling,
                                                         double accelerationScaling,
                                                         double planningTimeSec)
{
  vtkMoveitMsgsRobotTrajectory* traj = vtkMoveitMsgsRobotTrajectory::New();

  if (!mMRMLROS2Node || !mMRMLROS2Node->mInternals || !mMRMLROS2Node->mInternals->mNodePointer) {
    vtkErrorMacro(<< "PlanMoveItTrajectory: ROS2 node is not initialized");
    return traj;
  }
  if (groupName.empty()) {
    vtkErrorMacro(<< "PlanMoveItTrajectory: groupName is empty");
    return traj;
  }

  auto node = mMRMLROS2Node->mInternals->mNodePointer;
  moveit::planning_interface::MoveGroupInterface moveGroup(node, groupName);

  const auto jointNames = moveGroup.getJointNames();
  if (jointNames.size() != goalJointValues.size()) {
    vtkErrorMacro(<< "PlanMoveItTrajectory: expected " << jointNames.size()
                  << " joint values for group '" << groupName << "' but got " << goalJointValues.size());
    return traj;
  }

  const double velScale = std::clamp(velocityScaling, 0.0, 1.0);
  const double accScale = std::clamp(accelerationScaling, 0.0, 1.0);
  moveGroup.setMaxVelocityScalingFactor(velScale);
  moveGroup.setMaxAccelerationScalingFactor(accScale);
  moveGroup.setPlanningTime(planningTimeSec > 0.0 ? planningTimeSec : 2.0);

  // Set start state to current robot state from planning scene
  moveGroup.setStartStateToCurrentState();

  std::map<std::string, double> targets;
  for (size_t i = 0; i < jointNames.size(); ++i) {
    targets[jointNames[i]] = goalJointValues[i];
  }

  moveGroup.setJointValueTarget(targets);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = moveGroup.plan(plan);
  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    vtkROS2ToSlicer(plan.trajectory, vtkSmartPointer<vtkMoveitMsgsRobotTrajectory>(traj));
  } else {
    vtkErrorMacro(<< "PlanMoveItTrajectory: planning failed for group '" << groupName
                  << "' with MoveItErrorCode=" << result.val);
  }

  return traj;
}

std::string vtkMRMLROS2RobotNode::PlanMoveItTrajectoryJSON(const std::string& groupName,
                                                           const std::vector<double>& goalJointValues,
                                                           double velocityScaling,
                                                           double accelerationScaling,
                                                           double planningTimeSec)
{
  auto traj = PlanMoveItTrajectory(groupName, goalJointValues, velocityScaling, accelerationScaling, planningTimeSec);
  
  moveit_msgs::msg::RobotTrajectory ros_traj;
  vtkSlicerToROS2(traj, ros_traj, mMRMLROS2Node->mInternals->mNodePointer);

  // Cache the trajectory for later execution
  mInternals->CachedTrajectory = ros_traj;
  
  if (ros_traj.joint_trajectory.points.empty()) {
    traj->Delete();
    return "{}";  // Empty JSON on failure
  }

  // Build JSON manually to avoid extra dependencies
  std::ostringstream json;
  json << "{\"joint_names\":[";
  
  for (size_t i = 0; i < ros_traj.joint_trajectory.joint_names.size(); ++i) {
    if (i > 0) json << ",";
    json << "\"" << ros_traj.joint_trajectory.joint_names[i] << "\"";
  }
  
  json << "],\"points\":[";
  
  for (size_t i = 0; i < ros_traj.joint_trajectory.points.size(); ++i) {
    const auto& pt = ros_traj.joint_trajectory.points[i];
    if (i > 0) json << ",";
    
    json << "{\"positions\":[";
    for (size_t j = 0; j < pt.positions.size(); ++j) {
      if (j > 0) json << ",";
      json << pt.positions[j];
    }
    
    json << "],\"velocities\":[";
    for (size_t j = 0; j < pt.velocities.size(); ++j) {
      if (j > 0) json << ",";
      json << pt.velocities[j];
    }
    
    json << "],\"accelerations\":[";
    for (size_t j = 0; j < pt.accelerations.size(); ++j) {
      if (j > 0) json << ",";
      json << pt.accelerations[j];
    }
    
    json << "],\"time_from_start\":" << pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9 << "}";
  }
  
  json << "]}";
  return json.str();
}

bool vtkMRMLROS2RobotNode::ExecuteMoveItTrajectory(const std::string& groupName, vtkMoveitMsgsRobotTrajectory* trajectory)
{
  if (!mMRMLROS2Node || !mMRMLROS2Node->mInternals || !mMRMLROS2Node->mInternals->mNodePointer) {
    vtkErrorMacro(<< "ExecuteMoveItTrajectory: ROS2 node is not initialized");
    return false;
  }
  if (groupName.empty()) {
    vtkErrorMacro(<< "ExecuteMoveItTrajectory: groupName is empty");
    return false;
  }
  if (!trajectory) {
    vtkErrorMacro(<< "ExecuteMoveItTrajectory: trajectory is null");
    return false;
  }
  
  try {
    auto node = mMRMLROS2Node->mInternals->mNodePointer;
    
    moveit_msgs::msg::RobotTrajectory ros_traj;
    vtkSlicerToROS2(trajectory, ros_traj, node);

    if (ros_traj.joint_trajectory.points.empty()) {
      vtkErrorMacro(<< "ExecuteMoveItTrajectory: trajectory is empty");
      return false;
    }

    moveit::planning_interface::MoveGroupInterface moveGroup(node, groupName);

    // Create a plan with the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory = ros_traj;

    // Execute the trajectory
    auto result = moveGroup.execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      vtkInfoMacro(<< "ExecuteMoveItTrajectory: Successfully executed trajectory for group '" << groupName << "'");
      return true;
    } else {
      vtkErrorMacro(<< "ExecuteMoveItTrajectory: Execution failed for group '" << groupName 
                    << "' with MoveItErrorCode=" << result.val);
      return false;
    }
  }
  catch (const std::exception& e) {
    vtkErrorMacro(<< "ExecuteMoveItTrajectory: exception - " << e.what());
    return false;
  }
}

bool vtkMRMLROS2RobotNode::ExecuteCachedMoveItTrajectory(const std::string& groupName)
{
  if (mInternals->CachedTrajectory.joint_trajectory.points.empty()) {
    vtkErrorMacro(<< "ExecuteCachedMoveItTrajectory: No cached trajectory available. Call PlanMoveItTrajectoryJSON first.");
    return false;
  }

  vtkSmartPointer<vtkMoveitMsgsRobotTrajectory> vtk_traj = vtkSmartPointer<vtkMoveitMsgsRobotTrajectory>::New();
  vtkROS2ToSlicer(mInternals->CachedTrajectory, vtk_traj);
  return ExecuteMoveItTrajectoryAsync(groupName, vtk_traj.Get());
}

bool vtkMRMLROS2RobotNode::PlanAndExecuteMoveItTrajectory(const std::string& groupName,
                                                           const std::vector<double>& goalJointValues,
                                                           double velocityScaling,
                                                           double accelerationScaling,
                                                           double planningTimeSec)
{
  // First plan the trajectory
  auto trajectory = PlanMoveItTrajectory(groupName, goalJointValues, velocityScaling, accelerationScaling, planningTimeSec);
  
  moveit_msgs::msg::RobotTrajectory ros_traj;
  vtkSlicerToROS2(trajectory, ros_traj, mMRMLROS2Node->mInternals->mNodePointer);

  if (ros_traj.joint_trajectory.points.empty()) {
    vtkErrorMacro(<< "PlanAndExecuteMoveItTrajectory: Planning failed, cannot execute");
    trajectory->Delete();
    return false;
  }

  // Then execute it
  bool result = ExecuteMoveItTrajectory(groupName, trajectory);
  trajectory->Delete();
  return result;
}

bool vtkMRMLROS2RobotNode::ExecuteMoveItTrajectoryAsync(const std::string& groupName,
                                                        vtkMoveitMsgsRobotTrajectory* trajectory)
{
  if (!trajectory) {
    vtkErrorMacro(<< "ExecuteMoveItTrajectoryAsync: trajectory is null");
    return false;
  }
  
  moveit_msgs::msg::RobotTrajectory ros_traj;
  vtkSlicerToROS2(trajectory, ros_traj, mMRMLROS2Node->mInternals->mNodePointer);

  if (ros_traj.joint_trajectory.points.empty()) {
    vtkErrorMacro(<< "ExecuteMoveItTrajectoryAsync: trajectory is empty");
    return false;
  }

  // Launch execution in a background thread to avoid blocking UI
      auto nodePtr = mMRMLROS2Node;
    std::thread executionThread([this, nodePtr, groupName, ros_traj]() {
      try {
        auto node = nodePtr->mInternals->mNodePointer;
        moveit::planning_interface::MoveGroupInterface moveGroup(node, groupName);

        // Create a plan with the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory = ros_traj;

        // Execute the trajectory
        moveGroup.execute(plan);
      }
    catch (const std::exception& e) {
      vtkErrorMacro(<< "ExecuteMoveItTrajectoryAsync: exception in background thread - " << e.what());
    }
  });
  
  // Detach thread so it runs independently
  executionThread.detach();
  
  return true;
}


void vtkMRMLROS2RobotNode::UpdateScene(vtkMRMLScene *scene)
{
  Superclass::UpdateScene(scene);
  int nbNodeRefs = this->GetNumberOfNodeReferences("node");
  if (nbNodeRefs == 0) {
    // assigned to the default ROS node
    auto defaultNode = scene->GetFirstNodeByName("ros2:node:slicer");
    auto nodeId = defaultNode->GetID();
    if(!defaultNode) {
      vtkErrorMacro(<< "UpdateScene: default ros2 node unavailable. Unable to set reference for broadcaster \"" << GetName() << "\"");
      return;
    }
    defaultNode->SetNthNodeReferenceID("robot", defaultNode->GetNumberOfNodeReferences("robot"),this->GetID());
    this->SetNodeReferenceID("node", nodeId);
  } else if (nbNodeRefs == 1) {
    auto defaultNode = scene->GetFirstNodeByName("ros2:node:slicer");
    auto nodeId = defaultNode->GetID();
    defaultNode->SetNthNodeReferenceID("robot", defaultNode->GetNumberOfNodeReferences("robot"), this->GetID());
    this->SetNodeReferenceID("node", nodeId);
  } else {
    vtkErrorMacro(<< "UpdateScene: more than one ROS2 node reference defined for broadcaster \"" << GetName() << "\"");
  }
}
