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
#include "vtkCubeSource.h"

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
  // TODO: this should be moved over from widget class (Figure out library linking issue)
  // Parser the urdf file into a KDL tree -
  // KDL::Tree my_tree;
  // if (!kdl_parser::treeFromFile("/home/laura/ros2_ws/src/slicer_ros/models/omni.urdf", my_tree)){
  //   return; //std::cerr << "No urdf file to load." << filename << std::endl;
  // }
  // Keeping this as an example of how to pass to the function
  std::cerr << "hello" << filename << std::endl;


  // Instantiate the vtkCubeSource as a place holder while I figure out how to load an stl model
  vtkNew< vtkCubeSource > cube;
  cube->SetXLength(10);
  cube->SetYLength(10);
  cube->SetZLength(10);
  cube->Update();

  // Add a model node to the scene
  vtkMRMLModelNode *modelNodeToUpdate  = nullptr;
  vtkNew< vtkMRMLModelNode > modelNode;
  this->GetMRMLScene()->AddNode( modelNode.GetPointer() );
  modelNode->SetName( "CubeModel" );
  modelNodeToUpdate = modelNode.GetPointer();
  modelNodeToUpdate->SetAndObservePolyData( cube->GetOutput() );

  // Set the model display node to visible
  if (modelNodeToUpdate->GetDisplayNode() == NULL)
  {
    vtkNew< vtkMRMLModelDisplayNode > modelDisplayNode;
    this->GetMRMLScene()->AddNode( modelDisplayNode.GetPointer() );
    modelDisplayNode->SetName( "NeedleModelDisplay" );
    modelDisplayNode->SetColor( 0.0, 1.0, 1.0 );
    modelNodeToUpdate->SetAndObserveDisplayNodeID( modelDisplayNode->GetID() );
    modelDisplayNode->SetAmbient( 0.2 );
    modelDisplayNode->SetScalarVisibility(1);

  }

  // Create a transform node to add the model to so we can move it around
  vtkNew<vtkMRMLTransformStorageNode> storageNode;

  vtkSmartPointer<vtkMRMLTransformNode> tnode;

	storageNode->SetScene(this->GetMRMLScene());

 	vtkNew<vtkMRMLTransformNode> generalTransform;
 	generalTransform->SetScene(this->GetMRMLScene());
  tnode = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
 	storageNode->ReadData(tnode.GetPointer());
  tnode->SetName("TransformNode");
  this->GetMRMLScene()->AddNode(storageNode.GetPointer());
 	this->GetMRMLScene()->AddNode(tnode);
  tnode->SetAndObserveStorageNodeID(storageNode->GetID());
  // Other TODO: initialize the model node with something so it doesn't add a null (see if I can change the position of it/ set visibility)

  modelNode->SetAndObserveTransformNodeID(tnode->GetID());
}
