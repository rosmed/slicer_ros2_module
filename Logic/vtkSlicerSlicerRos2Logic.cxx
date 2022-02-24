/*==============================================================================

  Program: 3D Slicer

  Portions (c) Copyright Brigham and Women's Hospital (BWH) All Rights Reserved.

  See COPYRIGHT.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

==============================================================================*/

// SlicerRos2 Logic includes
#include "vtkSlicerSlicerRos2Logic.h"

// MRML includes
#include <vtkMRMLScene.h>
#include <vtkMRMLModelNode.h>
#include <vtkMRMLTransformStorageNode.h>
#include <vtkMRMLTransformNode.h>
#include <vtkMRMLLinearTransformNode.h>

// VTK includes
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>

#include <qSlicerCoreIOManager.h>

// KDL include_directories
#include "kdl_parser/kdl_parser.hpp"
// #include <iostream>
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <urdf/model.h>

// Python includes
#include "vtkSlicerConfigure.h"
#ifdef Slicer_USE_PYTHONQT
#include "PythonQt.h"
#endif

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlicerSlicerRos2Logic);

//----------------------------------------------------------------------------
vtkSlicerSlicerRos2Logic::vtkSlicerSlicerRos2Logic()
{
}

//----------------------------------------------------------------------------
vtkSlicerSlicerRos2Logic::~vtkSlicerSlicerRos2Logic()
{
}

//----------------------------------------------------------------------------
void vtkSlicerSlicerRos2Logic::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//---------------------------------------------------------------------------
void vtkSlicerSlicerRos2Logic::SetMRMLSceneInternal(vtkMRMLScene * newScene)
{
  vtkNew<vtkIntArray> events;
  events->InsertNextValue(vtkMRMLScene::NodeAddedEvent);
  events->InsertNextValue(vtkMRMLScene::NodeRemovedEvent);
  events->InsertNextValue(vtkMRMLScene::EndBatchProcessEvent);
  this->SetAndObserveMRMLSceneEventsInternal(newScene, events.GetPointer());
}

//-----------------------------------------------------------------------------
void vtkSlicerSlicerRos2Logic::RegisterNodes()
{
  assert(this->GetMRMLScene() != 0);
}

//---------------------------------------------------------------------------
void vtkSlicerSlicerRos2Logic::UpdateFromMRMLScene()
{
  assert(this->GetMRMLScene() != 0);
}

//---------------------------------------------------------------------------
void vtkSlicerSlicerRos2Logic
::OnMRMLSceneNodeAdded(vtkMRMLNode* vtkNotUsed(node))
{
}

//---------------------------------------------------------------------------
void vtkSlicerSlicerRos2Logic
::OnMRMLSceneNodeRemoved(vtkMRMLNode* vtkNotUsed(node))
{
}

//----------------------------------------------------------------------------
void vtkSlicerSlicerRos2Logic
::loadRobotSTLModels(const std::string& filename)
{
  // Parser the urdf file into an urdf model - to get names of links and pos/ rpy
  urdf::Model my_model;
  if (!my_model.initFile(filename)) {
      return;
  }

  // load urdf file into a kdl tree to do forward kinematics
  KDL::Tree my_tree;
  if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)) {
    return; //std::cout << "No urdf file to load." << filename << std::endl;
  }

  // Start by getting the name of the root link and add it to the vector of strings
  std::shared_ptr<const urdf::Link> root = my_model.getRoot();
  std::string root_name = root->name;
  std::vector<std::string> link_names_vector;
  link_names_vector.push_back(root_name);
  std::vector< std::shared_ptr< urdf::Visual > > visual_vector;
  visual_vector.push_back(root->visual);


  while (true) {
    std::shared_ptr<const urdf::Link> current_link = my_model.getLink(link_names_vector[link_names_vector.size() - 1]);
    std::vector< std::shared_ptr< urdf::Link > > child_link =  current_link->child_links;

    if (child_link.size() == 0) {
      break;
    } else {
      for (std::shared_ptr<urdf::Link> i: child_link) {
	link_names_vector.push_back(i->name);
	visual_vector.push_back(i->visual); // need to get the origin from the visual
      }
    }
  }

  // Print out the list of link names
  for (std::string i: link_names_vector) {
    std::cout << "[" << i << "] ";
  }
  std::cout << std::endl;

  //Call load STL model functions with python - can't find C++ implementation
  QList<QVariant> link_names_for_loading;
  // Link names need to be converted to QList of QVariants to be passed to python script
  for (size_t j = 0; j < link_names_vector.size(); j ++) {
    QVariant link_to_be_added;
    link_to_be_added = QString::fromStdString(link_names_vector[j]);
    link_names_for_loading.append(link_to_be_added);
  }
  //link_names_for_loading.append(link_names_vector);
  #ifdef Slicer_USE_PYTHONQT
    PythonQt::init();
    PythonQtObjectPtr context = PythonQt::self()->getMainModule();
    context.addVariable("linkNamesList", link_names_for_loading);
    context.evalScript(QString(
    "import slicer \n"
    "from pathlib import Path \n"
    "print(linkNamesList) \n"
    "mesh_dir = str(Path.home()) + '/ros2_ws/src/SlicerRos2/models/meshes/' \n"
    "for j in range(len(linkNamesList)): \n"
    " slicer.util.loadModel(mesh_dir + linkNamesList[j] + '.stl') \n" ));
  #endif

  KDL::Chain kdl_chain;
  std::string base_frame(link_names_vector[0]); // Specify the base to tip you want ie. joint 1 to 2 (base to torso)
  std::string tip_frame(link_names_vector[link_names_vector.size() - 1]);
  if (!my_tree.getChain(base_frame, tip_frame, kdl_chain)) {
    std::cerr << "not working" << std::endl;
    return;
  }
  mKDLChainSize = kdl_chain.getNrOfSegments();
  std::cout << "The chain has " << mKDLChainSize
	    << " segments" << std::endl
	    << "Found " << link_names_vector.size()
	    << " links" << std::endl;

  std::cout << "This is the joint position array" << std::endl;


  // Initialize the fk solver
  mKDLSolver = new KDL::ChainFkSolverPos_recursive(kdl_chain);

  // Allocate array for links transformation nodes
  mChainNodeTransforms.resize(mKDLChainSize);

  // Create a vtkMRMLTransform Node for each of these frames
  for (size_t l = 0; l < mKDLChainSize; l++) {
    vtkNew<vtkMRMLTransformStorageNode> storageNode;
    storageNode->SetScene(this->GetMRMLScene());
    vtkNew<vtkMRMLTransformNode> generalTransform;
    generalTransform->SetScene(this->GetMRMLScene());
    mChainNodeTransforms[l] = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
    storageNode->ReadData(mChainNodeTransforms[l].GetPointer());
    mChainNodeTransforms[l]->SetName((link_names_vector[l + 1] + "_transform").c_str());
    this->GetMRMLScene()->AddNode(storageNode.GetPointer());
    this->GetMRMLScene()->AddNode(mChainNodeTransforms[l]);
    mChainNodeTransforms[l]->SetAndObserveStorageNodeID(storageNode->GetID());
  }

  std::vector<double> initialJointValues(mKDLChainSize, 0.0);
  initialJointValues[1] = 0.8; // for testing
  UpdateFK(initialJointValues);

  // Get the origin and rpy
  std::vector<urdf::Pose> origins;
  for (std::shared_ptr<urdf::Visual> i: visual_vector) {
    urdf::Pose origin;
    origin = i->origin;
    origins.push_back(origin);
  }

  // Set up the initial position for each link (Rotate and Translate based on origin and rpy from the urdf file)
  for (size_t k = 0; k < (mKDLChainSize + 1); k ++) {
    vtkNew<vtkMRMLTransformStorageNode> storageNode;
    vtkSmartPointer<vtkMRMLTransformNode> tnode;
    storageNode->SetScene(this->GetMRMLScene());
    vtkNew<vtkMRMLTransformNode> generalTransform;
    generalTransform->SetScene(this->GetMRMLScene());
    tnode = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
    storageNode->ReadData(tnode.GetPointer());
    tnode->SetName("InitialPosition");
    this->GetMRMLScene()->AddNode(storageNode.GetPointer());
    this->GetMRMLScene()->AddNode(tnode);
    tnode->SetAndObserveStorageNodeID(storageNode->GetID());

    vtkTransform * modifiedTransform = vtkTransform::SafeDownCast(tnode->GetTransformToParent());
    urdf::Pose origin = origins[k];
    modifiedTransform->Translate(origin.position.x, origin.position.y, origin.position.z);
    tnode->SetAndObserveTransformToParent(modifiedTransform);
    tnode->Modified();
    vtkTransform * modifiedTransform2 = vtkTransform::SafeDownCast(tnode->GetTransformToParent());
    double r = 0.0;
    double p = 0.0;
    double y = 0.0;
    origin.rotation.getRPY(r, p, y);
    modifiedTransform2->RotateZ(y * (180.0/M_PI)); // RAD to degree conversion - use math.pi instead
    modifiedTransform2->RotateY(p * (180.0/M_PI));
    modifiedTransform2->RotateX(r * (180.0/M_PI));
    tnode->SetAndObserveTransformToParent(modifiedTransform2);
    tnode->Modified();

    vtkNew<vtkMatrix4x4> initialPositionMatrix;
    tnode->GetMatrixTransformToParent(initialPositionMatrix);

    // Apply LPS to RAS conversion
    vtkNew<vtkMatrix4x4> LPSToRAS_matrix;
    LPSToRAS_matrix->SetElement(0, 0, -1.0);
    LPSToRAS_matrix->SetElement(1, 1, -1.0);

    LPSToRAS_matrix->Multiply4x4(initialPositionMatrix, LPSToRAS_matrix, LPSToRAS_matrix);
    tnode->SetMatrixTransformToParent(LPSToRAS_matrix);
    tnode->Modified();

    if (k == 0) {
      vtkMRMLModelNode *modelNode = vtkMRMLModelNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names_vector[k].c_str()));
      assert(modelNode);
      modelNode->SetAndObserveTransformNodeID(tnode->GetID());
    } else {
      vtkMRMLTransformNode *transformNode = vtkMRMLTransformNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName((link_names_vector[k] + "_transform").c_str()));
      tnode->SetAndObserveTransformNodeID(transformNode->GetID());

      vtkMRMLModelNode *modelNode = vtkMRMLModelNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names_vector[k].c_str()));
      modelNode->SetAndObserveTransformNodeID(tnode->GetID());
    }
  }
}

void vtkSlicerSlicerRos2Logic::UpdateFK(const std::vector<double> & jointValues)
{
  // make sure the solver exists
  if (!mKDLSolver) {
    std::cout << "FK solver not initialized" << std::endl;
    return;
  }
  // make sure number of joint values is correct
  if (jointValues.size() != mKDLChainSize) {
    std::cout << "FK solver expects " << mKDLChainSize
	      << " values but UpdateFK was called with a vector of size "
	      << jointValues.size() << std::endl;
    return;
  }

  // Set up an std vector of frames
  std::vector<KDL::Frame> FK_frames;
  FK_frames.resize(mKDLChainSize);

  // Convert std::vector to KDL joint array
  auto jointArray = KDL::JntArray(mKDLChainSize);
  for (size_t index; index < mKDLChainSize; ++index) {
    jointArray(index) = jointValues[index];
  }

  // Calculate forward position kinematics
  mKDLSolver->JntToCart(jointArray, FK_frames);

  //Get the matrix and update it based on the forward kinematics
  for (size_t l = 0; l < mKDLChainSize; l++) {
    KDL::Frame cartpos;
    cartpos = FK_frames[l];
    vtkNew<vtkMatrix4x4> matrix;
    for (size_t i = 0; i < 4; i++) {
      for (size_t j=0; j <4; j ++) {
        matrix->SetElement(i,j, cartpos(i,j));
      }
    }
    // Update the matrix for the transform
    mChainNodeTransforms[l]->SetMatrixTransformToParent(matrix);
    mChainNodeTransforms[l]->Modified();
  }
}
