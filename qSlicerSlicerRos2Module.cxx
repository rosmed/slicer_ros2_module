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
#include <vtkSlicerSlicerRos2Logic.h>

// SlicerRos2 includes
#include "qSlicerSlicerRos2Module.h"
#include "qSlicerSlicerRos2ModuleWidget.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerSlicerRos2ModulePrivate
{
public:
  qSlicerSlicerRos2ModulePrivate();
};

//-----------------------------------------------------------------------------
// qSlicerSlicerRos2ModulePrivate methods

//-----------------------------------------------------------------------------
qSlicerSlicerRos2ModulePrivate::qSlicerSlicerRos2ModulePrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerSlicerRos2Module methods

//-----------------------------------------------------------------------------
qSlicerSlicerRos2Module::qSlicerSlicerRos2Module(QObject* _parent)
  : Superclass(_parent)
  , d_ptr(new qSlicerSlicerRos2ModulePrivate)
{
}

//-----------------------------------------------------------------------------
qSlicerSlicerRos2Module::~qSlicerSlicerRos2Module()
{
}

//-----------------------------------------------------------------------------
QString qSlicerSlicerRos2Module::helpText() const
{
  return "This is a loadable module that can be bundled in an extension";
}

//-----------------------------------------------------------------------------
QString qSlicerSlicerRos2Module::acknowledgementText() const
{
  return "This work was partially funded by NIH grant NXNNXXNNNNNN-NNXN";
}

//-----------------------------------------------------------------------------
QStringList qSlicerSlicerRos2Module::contributors() const
{
  QStringList moduleContributors;
  moduleContributors << QString("John Doe (AnyWare Corp.)");
  return moduleContributors;
}

//-----------------------------------------------------------------------------
QIcon qSlicerSlicerRos2Module::icon() const
{
  return QIcon(":/Icons/SlicerRos2.png");
}

//-----------------------------------------------------------------------------
QStringList qSlicerSlicerRos2Module::categories() const
{
  return QStringList() << "Examples";
}

//-----------------------------------------------------------------------------
QStringList qSlicerSlicerRos2Module::dependencies() const
{
  return QStringList();
}

//-----------------------------------------------------------------------------
void qSlicerSlicerRos2Module::setup()
{
  this->Superclass::setup();
}

//-----------------------------------------------------------------------------
qSlicerAbstractModuleRepresentation* qSlicerSlicerRos2Module
::createWidgetRepresentation()
{
  return new qSlicerSlicerRos2ModuleWidget;
}

//-----------------------------------------------------------------------------
vtkMRMLAbstractLogic* qSlicerSlicerRos2Module::createLogic()
{
  return vtkSlicerSlicerRos2Logic::New();
}
