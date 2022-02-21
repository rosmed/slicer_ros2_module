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
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <urdf/model.h>
#include <boost/function.hpp>

#include <stdlib.h>     /* getenv, to be removed later */
#include <boost/shared_ptr.hpp>

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
  char * pHome = getenv ("HOME");
  const std::string home(pHome);


  // Parser the urdf file into an urdf model - to get names of links and pos/ rpy
  urdf::Model my_model;
  if (!my_model.initFile(filename)){
      return;
  }

  // load urdf file into a kdl tree to do forward kinematics
  KDL::Tree my_tree;
  if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
    return; //std::cerr << "No urdf file to load." << filename << std::endl;
  }

  // Start by getting the name of the root link and add it to the vector of strings
  std::shared_ptr<const urdf::Link> root = my_model.getRoot();
  std::string root_name = root->name.c_str();
  std::vector<std::string> link_names_vector;
  link_names_vector.push_back(root_name);
  std::vector< std::shared_ptr< urdf::Visual > > visual_vector;
  visual_vector.push_back(root->visual);


  for (int j= 0; j < 50; j ++){ // 50 is an arbitrary choice because we don't know how many links

    std::shared_ptr<const urdf::Link> current_link = my_model.getLink(link_names_vector[link_names_vector.size() - 1]);
    std::vector< std::shared_ptr< urdf::Link > > child_link =  current_link->child_links;

    if (child_link.size() == 0){
      break;
    }
    else{
      for (std::shared_ptr< urdf::Link > i: child_link){
          std::string child_name = i->name.c_str();
          link_names_vector.push_back(child_name);
          visual_vector.push_back(i->visual); // need to get the origin from the visual
        }
      }
  }

  // Print out the list of link names
  for (std::string i: link_names_vector)
    std::cout << i << ' ';

  // Do some type conversion and name changes to make FK transform
  std::vector<std::string> link_names_FK_vector;
  std::string forwardKinematics = "_ForwardKin";
  for (int j = 0; j < link_names_vector.size(); j++)
    link_names_FK_vector.push_back(link_names_vector[j]);
  for (int j = 0; j < link_names_FK_vector.size(); j++)
    link_names_FK_vector[j].append(forwardKinematics);



  //Call load STL model functions with python - can't find C++ implementation
  QList<QVariant> link_names_for_loading;
  // Link names need to be converted to QList of QVariants to be passed to python script
  for (int j = 0; j < link_names_vector.size(); j ++){
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
  auto jointpositions = KDL::JntArray(nj);
  //How to print it
  for (size_t q = 0; q < nj; q++){
    std::cout << jointpositions.operator()(q) << std::endl;
    if (q == 1){
      jointpositions.operator()(q) = 0; // Upper arm angle in radians
    }
    std::cout << jointpositions.operator()(q) << std::endl;
  }

  // Initialize the fk solver
  auto fksolver = KDL::ChainFkSolverPos_recursive(kdl_chain);

  // Calculate forward position kinematics
  bool kinematics_status;
  kinematics_status = fksolver.JntToCart(jointpositions,FK_frames);
  if (kinematics_status) {
      std::cout << "Thanks KDL!" <<std::endl;
  } else {
      printf("%s \n","Error: could not calculate forward kinematics :(");
  }

  // Now we have an std vector of KDL frames with the correct kinematics
  std::cout << "After FK Solver" <<std::endl;
  for (KDL::Frame i: FK_frames)
    std::cout << i << ' ';

  // Get the origin and rpy

  std::vector<urdf::Pose> origins;
  for (std::shared_ptr< urdf::Visual > i: visual_vector){
    urdf::Pose origin;
    origin =  i->origin;
    origins.push_back(origin);
  }


  // Create a vtkMRMLTransform Node for each of these frames
  for (int l = 0; l < 6; l++){
    vtkNew<vtkMRMLTransformStorageNode> storageNode;
    vtkSmartPointer<vtkMRMLTransformNode> tnode;
    storageNode->SetScene(this->GetMRMLScene());
    vtkNew<vtkMRMLTransformNode> generalTransform;
    generalTransform->SetScene(this->GetMRMLScene());
    tnode = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
    storageNode->ReadData(tnode.GetPointer());
    tnode->SetName(link_names_FK_vector[l + 1].c_str());
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

  }

  std::cout << "Adding loop" <<std::endl;
  // Set up the initial position for each link (Rotate and Translate based on origin and rpy from the urdf file)
  for (int k = 0; k < 7; k ++){
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
    urdf::Pose origin = origins[k];
    modifiedTransform->Translate(origin.position.x, origin.position.y, origin.position.z);
    tnode->SetAndObserveTransformToParent(modifiedTransform);
    tnode->Modified();
    vtkTransform *modifiedTransform2 = vtkTransform::SafeDownCast(tnode->GetTransformToParent());
    double r = 0.0;
    double p = 0.0;
    double y = 0.0;
    origin.rotation.getRPY(r, p, y);
    modifiedTransform2->RotateZ(y*57.2958); // RAD to degree conversion
    modifiedTransform2->RotateY(p*57.2958);
    modifiedTransform2->RotateX(r*57.2958);
    tnode->SetAndObserveTransformToParent(modifiedTransform2);
    tnode->Modified();

    // Initialize the LPStoRAS transform
    vtkNew<vtkMRMLTransformStorageNode> storageNode3;
    vtkSmartPointer<vtkMRMLTransformNode> LPSToRAS;
    storageNode->SetScene(this->GetMRMLScene());
    vtkNew<vtkMRMLTransformNode> generalTransform3;
    generalTransform3->SetScene(this->GetMRMLScene());
    LPSToRAS = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
    storageNode3->ReadData(LPSToRAS.GetPointer());
    LPSToRAS->SetName("LPSToRAS");
    this->GetMRMLScene()->AddNode(storageNode3.GetPointer());
    this->GetMRMLScene()->AddNode(LPSToRAS);
    LPSToRAS->SetAndObserveStorageNodeID(storageNode3->GetID());

    vtkNew<vtkMatrix4x4> LPSToRAS_matrix;
    LPSToRAS_matrix->SetElement(0,0,-1);
    LPSToRAS_matrix->SetElement(1,1,-1);
    LPSToRAS->SetMatrixTransformToParent(LPSToRAS_matrix);
    LPSToRAS->Modified();


    if (k == 0){
      vtkMRMLModelNode *modelNode = vtkMRMLModelNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names_vector[k].c_str()));
      LPSToRAS->SetAndObserveTransformNodeID(tnode->GetID());
      modelNode->SetAndObserveTransformNodeID(LPSToRAS->GetID());
    }
    else{
      vtkMRMLTransformNode *transformNode = vtkMRMLTransformNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names_FK_vector[k].c_str()));
      tnode->SetAndObserveTransformNodeID(transformNode->GetID());

      LPSToRAS->SetAndObserveTransformNodeID(tnode->GetID());

      vtkMRMLModelNode *modelNode = vtkMRMLModelNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names_vector[k].c_str()));
      modelNode->SetAndObserveTransformNodeID(LPSToRAS->GetID());
    }
  }
}
