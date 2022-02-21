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

// Qt includes
#include <QDebug>
#include <QtGui>
#include <QButtonGroup>

// Slicer includes
#include "qSlicerSlicerRos2ModuleWidget.h"
#include "ui_qSlicerSlicerRos2ModuleWidget.h"

// reference to Logic
#include "vtkSlicerSlicerRos2Logic.h"

// Slicer includes
#include "vtkMRMLModelDisplayNode.h"
#include "vtkMRMLMarkupsDisplayNode.h"
#include "vtkMRMLDisplayNode.h"
#include "vtkMRMLModelNode.h"
#include "vtkMRMLMarkupsFiducialNode.h"
#include "vtkMRMLInteractionNode.h"
#include "vtkMRMLScene.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerSlicerRos2ModuleWidgetPrivate: public Ui_qSlicerSlicerRos2ModuleWidget
{

public:
  qSlicerSlicerRos2ModuleWidgetPrivate();
  vtkSlicerSlicerRos2Logic* logic() const;
};


//-----------------------------------------------------------------------------
qSlicerSlicerRos2ModuleWidgetPrivate::qSlicerSlicerRos2ModuleWidgetPrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerSlicerRos2ModuleWidget methods

//-----------------------------------------------------------------------------
qSlicerSlicerRos2ModuleWidget::qSlicerSlicerRos2ModuleWidget(QWidget* _parent)
  : Superclass( _parent )
  , d_ptr( new qSlicerSlicerRos2ModuleWidgetPrivate )
{
}

//-----------------------------------------------------------------------------
qSlicerSlicerRos2ModuleWidget::~qSlicerSlicerRos2ModuleWidget()
{
}

//-----------------------------------------------------------------------------
void qSlicerSlicerRos2ModuleWidget::setup()
{
  Q_D(qSlicerSlicerRos2ModuleWidget);
  d->setupUi(this);
  this->Superclass::setup();

  // Setup connection for the button
  this->connect(d->PrintTreeButton, SIGNAL(clicked(bool)), this, SLOT(onPrintTreeButton()));
  char * pHome = getenv ("HOME");
  const std::string home(pHome);
  const std::string path = home + "/ros2_ws/src/SlicerRos2/models/omni.urdf";
  d->fileSelector->addItem("Not selected");
  d->fileSelector->addItem(path.c_str());
  this->connect(d->fileSelector, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(onFileSelected()) );
}

void qSlicerSlicerRos2ModuleWidget::onPrintTreeButton()
{
  Q_D(qSlicerSlicerRos2ModuleWidget);
  this->Superclass::setup();

  //this->logic()->loadRobotSTLModels(); // This didn't work because it would call the base class which is vtkMRMlAbstractLogic
  // have to do the SafeDownCast
  vtkSlicerSlicerRos2Logic* logic = vtkSlicerSlicerRos2Logic::SafeDownCast(this->logic());
	if (!logic)
  {
    qWarning() << Q_FUNC_INFO << " failed: Invalid Slicer Ros2 logic";
 	   return;
	 }
  //logic->loadRobotSTLModels();

}

void qSlicerSlicerRos2ModuleWidget::onFileSelected()
{
  Q_D(qSlicerSlicerRos2ModuleWidget);
  this->Superclass::setup();

  //this->logic()->loadRobotSTLModels(); // This didn't work because it would call the base class which is vtkMRMlAbstractLogic
  // have to do the SafeDownCast
  vtkSlicerSlicerRos2Logic* logic = vtkSlicerSlicerRos2Logic::SafeDownCast(this->logic());
	if (!logic)
  {
    qWarning() << Q_FUNC_INFO << " failed: Invalid Slicer Ros2 logic";
 	   return;
	}
  char * pHome = getenv ("HOME");
  const std::string home(pHome);
  const std::string path = home + "/ros2_ws/src/SlicerRos2/models/omni.urdf";
  logic->loadRobotSTLModels(path);

}
