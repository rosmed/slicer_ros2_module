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

// .NAME vtkSlicerSlicerRos2Logic - slicer logic class for volumes manipulation
// .SECTION Description
// This class manages the logic associated with reading, saving,
// and changing propertied of the volumes


#ifndef __vtkSlicerSlicerRos2Logic_h
#define __vtkSlicerSlicerRos2Logic_h

// Forward declarations
namespace KDL {
  class ChainFkSolverPos_recursive;
}
class vtkMRMLTransformNode;

// Slicer includes
#include "vtkSlicerModuleLogic.h"

#include "vtkSlicerSlicerRos2ModuleLogicExport.h"

/// \ingroup Slicer_QtModules_ExtensionTemplate
class VTK_SLICER_SLICERROS2_MODULE_LOGIC_EXPORT vtkSlicerSlicerRos2Logic :
  public vtkSlicerModuleLogic
{
public:

  static vtkSlicerSlicerRos2Logic *New();
  vtkTypeMacro(vtkSlicerSlicerRos2Logic, vtkSlicerModuleLogic);
  void PrintSelf(ostream& os, vtkIndent indent) override;
  void loadRobotSTLModels(const std::string & filename); // Could also be protected friend ** ask Anton
  void UpdateFK(const std::vector<double> & joinValues);

protected:
  vtkSlicerSlicerRos2Logic();
  ~vtkSlicerSlicerRos2Logic() override;

  void SetMRMLSceneInternal(vtkMRMLScene* newScene) override;
  /// Register MRML Node classes to Scene. Gets called automatically when the MRMLScene is attached to this logic class.
  void RegisterNodes() override;
  void UpdateFromMRMLScene() override;
  void OnMRMLSceneNodeAdded(vtkMRMLNode* node) override;
  void OnMRMLSceneNodeRemoved(vtkMRMLNode* node) override;


private:

  vtkSlicerSlicerRos2Logic(const vtkSlicerSlicerRos2Logic&); // Not implemented
  void operator=(const vtkSlicerSlicerRos2Logic&); // Not implemented

  KDL::ChainFkSolverPos_recursive * mKDLSolver = 0;
  size_t mKDLChainSize = 0;
  std::vector<vtkSmartPointer<vtkMRMLTransformNode> > mChainNodeTransforms;
};

#endif
