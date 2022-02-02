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
  // this is crashing Slicer
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
  // vtkMRMLScene *scene = this->GetMRMLScene();
  // vtkMRMLModelNode *modelNode  = nullptr;
  // modelNode->SetAndObservePolyData(cube->GetOutput());
  // scene->AddNode(modelNode);
  std::cerr << "hello" << filename << std::endl;

  // Other TODO: initialize the model node with something so it doesn't add a null (see if I can change the position of it/ set visibility)
}
