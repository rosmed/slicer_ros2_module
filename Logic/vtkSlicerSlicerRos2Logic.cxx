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
using namespace std;
using namespace KDL;

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
  // TODO: this should be moved over from widget class (Figure out library linking issue)
  // Parser the urdf file into a KDL tree -
  KDL::Tree my_tree;
  if (!kdl_parser::treeFromFile("/home/laura/ros2_ws/src/slicer_ros/models/omni.urdf", my_tree)){
    return; //std::cerr << "No urdf file to load." << filename << std::endl;
  }

  //Initialize python so you can call load modules
  #ifdef Slicer_USE_PYTHONQT
    PythonQt::init();
    PythonQtObjectPtr context = PythonQt::self()->getMainModule();
    context.evalScript(QString(
    "import slicer \n"
    "slicer.util.loadModel(r'/home/laura/ros2_ws/src/SlicerRos2/models/meshes/base.stl') \n"
    "slicer.util.loadModel(r'/home/laura/ros2_ws/src/SlicerRos2/models/meshes/torso.stl') \n"
    "slicer.util.loadModel(r'/home/laura/ros2_ws/src/SlicerRos2/models/meshes/upper_arm.stl') \n"
    "slicer.util.loadModel(r'/home/laura/ros2_ws/src/SlicerRos2/models/meshes/lower_arm.stl') \n"));
  #endif


  const char *link_names[4] = { "base", "torso", "upper_arm", "lower_arm" };
  double link_translation_x[4] = {0, 0, 0.0075, 0};
  double link_translation_y[4] = {-0.02, 0, 0, 0};
  double link_translation_z[4] = { 0, 0.036, 0, 0};
  double link_rotation_x[4] = {0,-90, 0, 90};
  double link_rotation_y[4] = {0,0, 0, 0};
  double link_rotation_z[4] = {0, 0, 0, 0};

  // Solve the forward kinematics of the KDL tree
  for (int k = 0; k < 3; k++){
    KDL::Chain kdl_chain;
    std::string base_frame(link_names[k]); // Specify the base to tip you want ie. joint 1 to 2 (base to torso)
    std::string tip_frame(link_names[k + 1]);
    if (!my_tree.getChain(base_frame, tip_frame, kdl_chain))
    {
      std::cerr << "not working" << std::endl;
      return;
    }

    // TODO: replace the hard coded link names with something like this
    KDL::Joint kdl_joint = kdl_chain.getSegment(0).getJoint();
    KDL::Segment kdl_segment = kdl_chain.getSegment(0);
    std::string segmentName(kdl_segment.getName());
    std::cerr << segmentName << std::endl;

    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(kdl_chain);

    // Create joint array
    unsigned int nj = kdl_chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);

    // Create the frame that will contain the results
    KDL::Frame cartpos;
    float x = 0;

    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos <<std::endl;
        x = cartpos.operator()(2,3);
        std::cout << cartpos.operator()(2,3) <<std::endl;
        std::cout << "Thanks KDL!" <<std::endl;
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }

    // Create a transform node to add the model to so we can move it around
    vtkNew<vtkMRMLTransformStorageNode> storageNode;
    vtkSmartPointer<vtkMRMLTransformNode> tnode;

  	storageNode->SetScene(this->GetMRMLScene());

   	vtkNew<vtkMRMLTransformNode> generalTransform;
   	generalTransform->SetScene(this->GetMRMLScene());
    tnode = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
   	storageNode->ReadData(tnode.GetPointer());
    tnode->SetName("ForwardKinematics");
    this->GetMRMLScene()->AddNode(storageNode.GetPointer());
   	this->GetMRMLScene()->AddNode(tnode);
    tnode->SetAndObserveStorageNodeID(storageNode->GetID());

    // Create a second transform for position (translation and rotation)

    vtkNew<vtkMRMLTransformStorageNode> positionStorageNode;
    vtkSmartPointer<vtkMRMLTransformNode> ptnode;

  	positionStorageNode->SetScene(this->GetMRMLScene());
    vtkNew<vtkMRMLTransformNode> positionTransform;
   	positionTransform->SetScene(this->GetMRMLScene());
    ptnode = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
   	positionStorageNode->ReadData(ptnode.GetPointer());
    ptnode->SetName("InitialPosition");
    this->GetMRMLScene()->AddNode(positionStorageNode.GetPointer());
   	this->GetMRMLScene()->AddNode(ptnode);
    ptnode->SetAndObserveStorageNodeID(positionStorageNode->GetID());

    vtkTransform *modifiedTransform = vtkTransform::SafeDownCast(ptnode->GetTransformToParent());
    modifiedTransform->Translate(link_translation_x[k + 1], link_translation_y[k + 1], link_translation_z[k + 1]);
    modifiedTransform->RotateX(link_rotation_x[k + 1]);
    modifiedTransform->RotateY(link_rotation_y[k + 1]);
    modifiedTransform->RotateZ(link_rotation_z[k + 1]);
    ptnode->SetAndObserveTransformToParent(modifiedTransform);
    ptnode->Modified();

    //Stack the transforms
    tnode->SetAndObserveTransformNodeID(ptnode->GetID());

    // Add model for the joint to transform hiearchy - so it moves with the transform
    vtkMRMLModelNode *modelNode = vtkMRMLModelNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names[k + 1]));
    modelNode->SetAndObserveTransformNodeID(tnode->GetID());

    // Get the matrix and update it based on the forward kinematics
    vtkMatrix4x4 *matrix = vtkMatrix4x4::SafeDownCast(tnode->GetMatrixTransformToParent());
    for (int i = 0; i < 4; i++) {
      for (int j=0; j <4; j ++){
        matrix->SetElement(i,j, cartpos.operator()(i,j));
      }
    }
    // Update the matrix for the transform
    tnode->SetMatrixTransformToParent(matrix);
  }
}
