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
#include "qSlicerRos2ModuleWidget.h"
#include "ui_qSlicerRos2ModuleWidget.h"

// reference to Logic
#include "vtkSlicerRos2Logic.h"

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
class qSlicerRos2ModuleWidgetPrivate: public Ui_qSlicerRos2ModuleWidget
{

public:
  qSlicerRos2ModuleWidgetPrivate();
  vtkSlicerRos2Logic* logic() const;
};


//-----------------------------------------------------------------------------
qSlicerRos2ModuleWidgetPrivate::qSlicerRos2ModuleWidgetPrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerRos2ModuleWidget methods

//-----------------------------------------------------------------------------
qSlicerRos2ModuleWidget::qSlicerRos2ModuleWidget(QWidget* _parent)
  : Superclass( _parent )
  , d_ptr( new qSlicerRos2ModuleWidgetPrivate )
{
  this->mTimer = new QTimer();
  mTimer->setSingleShot(false);
  mTimer->setInterval(20); // 20 ms, 50Hz
  mTimer->start();
}

//-----------------------------------------------------------------------------
qSlicerRos2ModuleWidget::~qSlicerRos2ModuleWidget()
{
  mTimer->stop();
  delete this->mTimer;
}

//-----------------------------------------------------------------------------
void qSlicerRos2ModuleWidget::setup()
{
  Q_D(qSlicerRos2ModuleWidget);
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
  this->connect(d->clearSceneButton, SIGNAL(clicked(bool)), this, SLOT(onClearSceneSelected()));

  // Set up timer connections
  connect(mTimer, SIGNAL( timeout() ), this, SLOT( onTimerTimeOut() ));
}

void qSlicerRos2ModuleWidget::onFileSelected(const QString& text)
{
  Q_D(qSlicerRos2ModuleWidget);
  this->Superclass::setup();

  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
	if (!logic)
  {
    qWarning() << Q_FUNC_INFO << " failed: Invalid Slicer Ros2 logic";
 	   return;
	}

  // Check if the timer is on or off before setting up the robot
  if (timerOff == true){
    mTimer->start();
    timerOff = false;
  }

  logic->loadRobotSTLModels();

}


void qSlicerRos2ModuleWidget::onTimerTimeOut()
{
  Q_D(qSlicerRos2ModuleWidget);
  this->Superclass::setup();

  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid Slicer Ros2 logic";
    return;
  }
  logic->Spin();
}

void qSlicerRos2ModuleWidget::onClearSceneSelected()
{
  Q_D(qSlicerRos2ModuleWidget);
  this->Superclass::setup();

  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid Slicer Ros2 logic";
    return;
  }
  logic->Clear();

  // Stop the timer too if three are no more models in the scene
  if (timerOff == false){
    mTimer->stop();
    timerOff = true;
  }

}
