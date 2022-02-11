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
#include <vtkMRMLModelDisplayNode.h>
#include <vtkMRMLTransformStorageNode.h>
#include <vtkMRMLTransformNode.h>
#include <vtkMRMLLinearTransformNode.h>

// VTK includes
#include <vtkIntArray.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkMatrix4x4.h>
#include "vtkCubeSource.h"
#include <vtkTransform.h>

#include <qSlicerCoreIOManager.h>

// STD includes
#include <cassert>

// KDL include_directories
#include "kdl_parser/kdl_parser.hpp"
#include<iostream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <urdf/model.h>
#include <boost/function.hpp>
using namespace std;
using namespace KDL;
using namespace urdf;

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
::loadRobotSTLModels()
{

  // Parser the urdf file into an urdf model - to get names of links and pos/ rpy
  urdf::Model my_model;
  if (!my_model.initFile("/home/laura/ros2_ws/src/slicer_ros/models/omni.urdf")){
      return;
  }

  // load urdf file into a kdl tree to do forward kinematics
  KDL::Tree my_tree;
  if (!kdl_parser::treeFromFile("/home/laura/ros2_ws/src/slicer_ros/models/omni.urdf", my_tree)){
    return; //std::cerr << "No urdf file to load." << filename << std::endl;
  }

  //Call load STL model functions with python - can't find C++ implementation
  #ifdef Slicer_USE_PYTHONQT
    PythonQt::init();
    PythonQtObjectPtr context = PythonQt::self()->getMainModule();
    context.evalScript(QString(
    "import slicer \n"
    "slicer.util.loadModel(r'/home/laura/ros2_ws/src/SlicerRos2/models/meshes/base.stl') \n"
    "slicer.util.loadModel(r'/home/laura/ros2_ws/src/SlicerRos2/models/meshes/torso.stl') \n"
    "slicer.util.loadModel(r'/home/laura/ros2_ws/src/SlicerRos2/models/meshes/wrist.stl') \n"
    "slicer.util.loadModel(r'/home/laura/ros2_ws/src/SlicerRos2/models/meshes/upper_arm.stl') \n"
    "slicer.util.loadModel(r'/home/laura/ros2_ws/src/SlicerRos2/models/meshes/tip.stl') \n"
    "slicer.util.loadModel(r'/home/laura/ros2_ws/src/SlicerRos2/models/meshes/lower_arm.stl') \n"));
  #endif

  // Hard coded for now
  const char *link_names[6] = { "base", "torso", "upper_arm", "lower_arm", "wrist", "tip"};
  const char *link_names_FK[5] = { "forwardKin_torso", "forwardKin_upperarm", "forwardKin_lower_arm", "forwardKin_wrist", "forwardKin_tip"};
  double link_translation_x[6] = {0, 0, 0.0075, 0, 0, 0};
  double link_translation_y[6] = {0.02, 0.036, 0, 0, 0, 0}; // these two are kinda hacked
  double link_translation_z[6] = { 0, 0, 0, 0, 0, 0};
  double link_rotation_x[6] = {0,90, 0, 90, 180, -90}; // need to find out why angle values are wrong and direction changes
  double link_rotation_y[6] = {0, 0, 0, 0, 0, 0};
  double link_rotation_z[6] = {0, 0, 0, 0, 0, 0};
  // Note the order of trasnforms for each stl model is whats screwing stuff up (/ might need to translate the lower )



  KDL::Chain kdl_chain;
  std::string base_frame("base"); // Specify the base to tip you want ie. joint 1 to 2 (base to torso)
  std::string tip_frame("tip");
  if (!my_tree.getChain(base_frame, tip_frame, kdl_chain))
  {
    std::cerr << "not working" << std::endl;
    return;
  }
  unsigned int nj = kdl_chain.getNrOfSegments();
  std::cerr << "The chain has this many segments" << std::endl;
  std::cerr << nj << std::endl;

  // Set up an std vector of frames
  std::vector<KDL::Frame> FK_frames;
  FK_frames.resize(nj);
  for (KDL::Frame i: FK_frames)
    std::cout << i << ' ';


  std::cerr << "This is the joint position array" << std::endl;
  KDL::JntArray jointpositions = JntArray(nj);
  //How to print it
  for (int q = 0; q < nj; q++){
    std::cout << jointpositions.operator()(q) << std::endl;
    if (q == 1){
      jointpositions.operator()(q) = -1.0;
    }
    std::cout << jointpositions.operator()(q) << std::endl;
  }

  // Initialize the fk solver
  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(kdl_chain);

  // Calculate forward position kinematics
  bool kinematics_status;
  kinematics_status = fksolver.JntToCart(jointpositions,FK_frames);
  if(kinematics_status>=0){
      std::cout << "Thanks KDL!" <<std::endl;
  }else{
      printf("%s \n","Error: could not calculate forward kinematics :(");
  }

  // Now we have an std vector of KDL frames with the correct kinematics
  std::cout << "After FK Solver" <<std::endl;
  for (KDL::Frame i: FK_frames)
    std::cout << i << ' ';

  // Create a vtkMRMLTransform Node for each of these frames
  for (int l = 0; l < 5; l++){
    vtkNew<vtkMRMLTransformStorageNode> storageNode;
    vtkSmartPointer<vtkMRMLTransformNode> tnode;
    storageNode->SetScene(this->GetMRMLScene());
    vtkNew<vtkMRMLTransformNode> generalTransform;
    generalTransform->SetScene(this->GetMRMLScene());
    tnode = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
    storageNode->ReadData(tnode.GetPointer());
    tnode->SetName(link_names_FK[l]);
    this->GetMRMLScene()->AddNode(storageNode.GetPointer());
    this->GetMRMLScene()->AddNode(tnode);
    tnode->SetAndObserveStorageNodeID(storageNode->GetID());

    //Get the matrix and update it based on the forward kinematics
    KDL::Frame cartpos;
    cartpos = FK_frames[l];
    vtkMatrix4x4 *matrix = vtkMatrix4x4::SafeDownCast(tnode->GetMatrixTransformToParent());
    for (int i = 0; i < 4; i++) {
      for (int j=0; j <4; j ++){
        matrix->SetElement(i,j, cartpos.operator()(i,j));
      }
    }
    // Update the matrix for the transform
    tnode->SetMatrixTransformToParent(matrix);

    // vtkMRMLModelNode *modelNode = vtkMRMLModelNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names[l + 1]));
    // modelNode->SetAndObserveTransformNodeID(tnode->GetID());
  }

  // Set up the initial position for each link (Rotate and Translate based on origin and rpy from the urdf file)
  for (int k = 0; k < 6; k ++){
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

    vtkTransform *modifiedTransform = vtkTransform::SafeDownCast(tnode->GetTransformToParent());

    //TODO: this needs to be different (FK * (STL_r * STL_tr)) - I think the order is messing stuff up
    modifiedTransform->RotateZ(link_rotation_z[k]);
    modifiedTransform->RotateY(link_rotation_y[k]);
    modifiedTransform->RotateX(link_rotation_x[k]);
    modifiedTransform->Translate(link_translation_x[k], link_translation_y[k], link_translation_z[k]);
    tnode->SetAndObserveTransformToParent(modifiedTransform);
    tnode->Modified();

    if (k == 0){
      vtkMRMLModelNode *modelNode = vtkMRMLModelNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names[k]));
      modelNode->SetAndObserveTransformNodeID(tnode->GetID());
    }
    else{
      vtkMRMLTransformNode *transformNode = vtkMRMLTransformNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names_FK[k-1]));
      tnode->SetAndObserveTransformNodeID(transformNode->GetID());

      vtkMRMLModelNode *modelNode = vtkMRMLModelNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names[k]));
      modelNode->SetAndObserveTransformNodeID(tnode->GetID());
    }
  }
}
