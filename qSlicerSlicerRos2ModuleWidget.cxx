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
#include <QTimer>
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
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

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
  this->mTimer = new QTimer();
  mTimer->setSingleShot(false);
  mTimer->setInterval(1000); // 1 sec
  this->mTimerPeriodCount = 0;
}

//-----------------------------------------------------------------------------
qSlicerSlicerRos2ModuleWidget::~qSlicerSlicerRos2ModuleWidget()
{
  delete this->mTimer;
}

//-----------------------------------------------------------------------------
void qSlicerSlicerRos2ModuleWidget::setup()
{
  Q_D(qSlicerSlicerRos2ModuleWidget);
  d->setupUi(this);
  this->Superclass::setup();

  // Start the QComboBox with a generic string
  d->fileSelector->addItem("Not selected");

  // Get the home directory path
  char * pHome = getenv ("HOME");
  const std::string home(pHome);

  // Add the subsequent folders where the urdf file should be found
  const std::string paths = home + "/ros2_ws/src/SlicerRos2/models/urdf/";
  for (const auto & file : fs::directory_iterator(paths))
    d->fileSelector->addItem(file.path().c_str());

  this->connect(d->fileSelector, SIGNAL(currentTextChanged(const QString&)), this, SLOT(onFileSelected(const QString&)));

  // Set up timer connections
  connect(mTimer, SIGNAL( timeout() ), this, SLOT( onTimerTimeOut() ));
  this->connect(d->activeCheckBox, SIGNAL(toggled(bool)), this, SLOT(onTimerStarted(bool)));
}

void qSlicerSlicerRos2ModuleWidget::onFileSelected(const QString& text)
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

  // Convert input QString to std::string to pass to logic
  const std::string model_path = text.toStdString();;
  logic->loadRobotSTLModels(model_path);

}

void qSlicerSlicerRos2ModuleWidget::onTimerStarted(bool state)
{
  Q_D(qSlicerSlicerRos2ModuleWidget);
  this->Superclass::setup();

  if (state == true){
    mTimer->start();
  }
  else{
    mTimer->stop();
  }
}

void qSlicerSlicerRos2ModuleWidget::onTimerTimeOut()
{
  Q_D(qSlicerSlicerRos2ModuleWidget);
  this->Superclass::setup();

  vtkSlicerSlicerRos2Logic* logic = vtkSlicerSlicerRos2Logic::SafeDownCast(this->logic());
	if (!logic)
  {
    qWarning() << Q_FUNC_INFO << " failed: Invalid Slicer Ros2 logic";
 	   return;
	}
  qWarning() << Q_FUNC_INFO << "Timer going";

}
