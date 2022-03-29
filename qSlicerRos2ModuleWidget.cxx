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
#include <QCloseEvent>
#include <QButtonGroup>
#include <QWidget>
#include <QVBoxLayout>
#include <QLayout>

// Slicer includes
#include "qSlicerRos2ModuleWidget.h"
#include "ui_qSlicerRos2ModuleWidget.h"
#include "qSlicerApplication.h"

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
  connect(qSlicerApplication::application(), SIGNAL(lastWindowClosed()), this, SLOT(stopSound()));

  // Setup state / selection options
  QVBoxLayout *stateBoxLayout = new QVBoxLayout;
  stateBoxLayout->addWidget(topicLineEdit);
  topicLineEdit->setEnabled(false);
  d->stateWidgetGroupBox->setLayout(stateBoxLayout);
  this->connect(d->stateSelectionComboBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(onStateSelection(const QString&)));


  // THIS IS A REPEAT OF ABOVE CODE TO THE NEW FILE SELECTOR SO I DONT BREAK ANYTHING
  urdfFileSelector->addItem("Not selected");
  for (const auto & file : fs::directory_iterator(paths))
    urdfFileSelector->addItem(file.path().c_str());

  // Setup description / selection options
  QVBoxLayout *descriptionBoxLayout = new QVBoxLayout;
  descriptionBoxLayout->addWidget(urdfFileSelector);
  descriptionBoxLayout->addWidget(nodeLineEdit);
  descriptionBoxLayout->addWidget(paramLineEdit);
  urdfFileSelector->setEnabled(false);
  nodeLineEdit->setEnabled(false);
  paramLineEdit->setEnabled(false);
  d->descriptionWidgetGroupBox->setLayout(descriptionBoxLayout);
  this->connect(d->descriptionSelectionComboBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(onDescriptionSelection(const QString&)));

  // Set up signals / slots for dynamically loaded widgets
  // grab the global widget vars and set up their signals
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

void qSlicerRos2ModuleWidget::stopSound() // Shouldn't be on quit - look here: https://doc.qt.io/qt-5/qapplication.html
{
  std::cerr << "closing event" << std::endl;
  mTimer->stop();
  delete this->mTimer;
}

void qSlicerRos2ModuleWidget::onStateSelection(const QString& text)
{
  Q_D(qSlicerRos2ModuleWidget);

  // Add all the dynamic widgets already and figure out how to hide and show them dynamically
  if (text == "Tf2"){
    d->stateWidgetGroupBox->setTitle("Tf2 selected");
    topicLineEdit->setEnabled(false);
  }
  else if (text == "Topic"){
    d->stateWidgetGroupBox->setTitle("Topic selected");
    topicLineEdit->setEnabled(true);
  }
  else if (text == "Not selected"){
    d->stateWidgetGroupBox->setTitle("Not selected");
    topicLineEdit->setEnabled(false);
  }

}

void qSlicerRos2ModuleWidget::onDescriptionSelection(const QString& text) // Shouldn't be on quit - look here: https://doc.qt.io/qt-5/qapplication.html
{
  Q_D(qSlicerRos2ModuleWidget);

  if (text == "File"){
    d->descriptionWidgetGroupBox->setTitle("File selected");
    urdfFileSelector->setEnabled(true);
    nodeLineEdit->setEnabled(false);
    paramLineEdit->setEnabled(false);
  }
  else if (text == "Param"){
    d->descriptionWidgetGroupBox->setTitle("Param selected");
    urdfFileSelector->setEnabled(false);
    nodeLineEdit->setEnabled(true);
    paramLineEdit->setEnabled(true);
  }
  else if (text == "Not selected"){
    d->descriptionWidgetGroupBox->setTitle("Not selected");
    urdfFileSelector->setEnabled(false);
    nodeLineEdit->setEnabled(false);
    paramLineEdit->setEnabled(false);
  }
}
