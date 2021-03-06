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
#include <vtkSlicerRos2Logic.h>

// SlicerRos2 includes
#include "qSlicerRos2Module.h"
#include "qSlicerRos2ModuleWidget.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerRos2ModulePrivate
{
public:
  qSlicerRos2ModulePrivate();
};

//-----------------------------------------------------------------------------
// qSlicerRos2ModulePrivate methods

//-----------------------------------------------------------------------------
qSlicerRos2ModulePrivate::qSlicerRos2ModulePrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerRos2Module methods

//-----------------------------------------------------------------------------
qSlicerRos2Module::qSlicerRos2Module(QObject* _parent)
  : Superclass(_parent)
  , d_ptr(new qSlicerRos2ModulePrivate)
{
}

//-----------------------------------------------------------------------------
qSlicerRos2Module::~qSlicerRos2Module()
{
}

//-----------------------------------------------------------------------------
QString qSlicerRos2Module::helpText() const
{
  return "This is a loadable module that can be bundled in an extension";
}

//-----------------------------------------------------------------------------
QString qSlicerRos2Module::acknowledgementText() const
{
  return "This work was partially funded by NIH grant NXNNXXNNNNNN-NNXN";
}

//-----------------------------------------------------------------------------
QStringList qSlicerRos2Module::contributors() const
{
  QStringList moduleContributors;
  moduleContributors << QString("John Doe (AnyWare Corp.)");
  return moduleContributors;
}

//-----------------------------------------------------------------------------
QIcon qSlicerRos2Module::icon() const
{
  return QIcon(":/Icons/SlicerRos2.png");
}

//-----------------------------------------------------------------------------
QStringList qSlicerRos2Module::categories() const
{
  return QStringList() << "Examples";
}

//-----------------------------------------------------------------------------
QStringList qSlicerRos2Module::dependencies() const
{
  return QStringList();
}

//-----------------------------------------------------------------------------
void qSlicerRos2Module::setup()
{
  this->Superclass::setup();
}

//-----------------------------------------------------------------------------
qSlicerAbstractModuleRepresentation* qSlicerRos2Module
::createWidgetRepresentation()
{
  return new qSlicerRos2ModuleWidget;
}

//-----------------------------------------------------------------------------
vtkMRMLAbstractLogic* qSlicerRos2Module::createLogic()
{
  return vtkSlicerRos2Logic::New();
}
