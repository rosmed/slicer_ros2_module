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

// SlicerROS2 Logic includes
#include <vtkSlicerROS2Logic.h>

// SlicerROS2 includes
#include "qSlicerROS2Module.h"
#include "qSlicerROS2ModuleWidget.h"

//-----------------------------------------------------------------------------
// qSlicerROS2Module methods

//-----------------------------------------------------------------------------
qSlicerROS2Module::qSlicerROS2Module(QObject* _parent)
  : Superclass(_parent)
{
}

//-----------------------------------------------------------------------------
qSlicerROS2Module::~qSlicerROS2Module()
{
}

//-----------------------------------------------------------------------------
QString qSlicerROS2Module::helpText() const
{
  return "ROS 2 Slicer Module.  See https://github.com/rosmed/slicer_ros2_module";
}

//-----------------------------------------------------------------------------
QString qSlicerROS2Module::acknowledgementText() const
{
  return "This work was partially funded by National Institutes of Health (USA) grant R01EB020667, the National Sciences and Engineering Research Council of Canada and the Canadian Institutes of Health Research";
}

//-----------------------------------------------------------------------------
QStringList qSlicerROS2Module::contributors() const
{
  QStringList moduleContributors;
  moduleContributors << QString("Laura Connolly (Queen’s University, Kingston, Canada), Anton Deguet (Johns Hopkins University, Baltimore, USA)");
  return moduleContributors;
}

//-----------------------------------------------------------------------------
QIcon qSlicerROS2Module::icon() const
{
  return QIcon(":/Icons/SlicerROS2.png");
}

//-----------------------------------------------------------------------------
QStringList qSlicerROS2Module::categories() const
{
  return QStringList() << "IGT";
}

//-----------------------------------------------------------------------------
QStringList qSlicerROS2Module::dependencies() const
{
  return QStringList();
}

//-----------------------------------------------------------------------------
void qSlicerROS2Module::setup()
{
  this->Superclass::setup();
}

//-----------------------------------------------------------------------------
qSlicerAbstractModuleRepresentation* qSlicerROS2Module
::createWidgetRepresentation()
{
  return new qSlicerROS2ModuleWidget;
}

//-----------------------------------------------------------------------------
vtkMRMLAbstractLogic* qSlicerROS2Module::createLogic()
{
  return vtkSlicerROS2Logic::New();
}
