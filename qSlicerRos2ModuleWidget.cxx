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

  this->connect(d->clearSceneButton, SIGNAL(clicked(bool)), this, SLOT(onClearSceneSelected()));

  // Set up timer connections
  connect(mTimer, SIGNAL(timeout()), this, SLOT(onTimerTimeOut()));
  connect(qSlicerApplication::application(), SIGNAL(lastWindowClosed()), this, SLOT(stopSound()));

  // Setup state / selection options
  QVBoxLayout *stateBoxLayout = new QVBoxLayout;
  stateBoxLayout->addWidget(topicLineEdit);
  d->stateWidgetGroupBox->setLayout(stateBoxLayout);
  this->connect(d->stateSelectionComboBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(onStateSelection(const QString&)));

  // Setup description / selection options
  QVBoxLayout *descriptionBoxLayout = new QVBoxLayout;
  descriptionBoxLayout->addWidget(urdfFileSelector);
  // Button is a place holder
  descriptionBoxLayout->addWidget(printButton);
  printButton->setText("Selected urdf");

  descriptionBoxLayout->addWidget(nodeLineEdit);
  descriptionBoxLayout->addWidget(paramLineEdit);
  d->descriptionWidgetGroupBox->setLayout(descriptionBoxLayout);
  this->connect(d->descriptionSelectionComboBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(onDescriptionSelection(const QString&)));

  // Set up signals / slots for dynamically loaded widgets
  // Note: All of the QLineEdits are triggered by pressing enter in the edit box - the slot functions access the text that was entered themselves
  this->connect(topicLineEdit, SIGNAL(returnPressed()), this, SLOT(onTopicNameEntered()));
  this->connect(nodeLineEdit, SIGNAL(returnPressed()), this, SLOT(onNodeOrParameterNameEntered()));
  this->connect(paramLineEdit, SIGNAL(returnPressed()), this, SLOT(onNodeOrParameterNameEntered()));
  this->connect(printButton, SIGNAL(clicked(bool)), this, SLOT(onPrintButtonSelected()));
  // file dialog signals are weird so using the button as a place holder just so you can print the name of the file you selected

  // Set default, assuming defaults are:
  // - state if from tf
  // - model is from param
  d->stateWidgetGroupBox->hide();
  urdfFileSelector->hide();
  printButton->hide();
}

void qSlicerRos2ModuleWidget::onFileSelected(const QString& text)
{
  // Anton: do we know why we have these 2 lines?  These are also in most callback onXYZ methods...
  Q_D(qSlicerRos2ModuleWidget);
  this->Superclass::setup();

  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid Slicer Ros2 logic";
    return;
  }

  // Check if the timer is on or off before setting up the robot
  // Anton: is this still needed?
  if (timerOff == true){
    mTimer->start();
    timerOff = false;
  }
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
  if (timerOff == false) {
    mTimer->stop();
    timerOff = true;
  }

}

void qSlicerRos2ModuleWidget::stopSound() // Shouldn't be on quit - look here: https://doc.qt.io/qt-5/qapplication.html
{
  std::cerr << "closing event" << std::endl;
  mTimer->stop();
}

void qSlicerRos2ModuleWidget::onStateSelection(const QString& text)
{
  Q_D(qSlicerRos2ModuleWidget);

  if (text == "tf2") {
    d->stateWidgetGroupBox->hide();
  } else if (text == "topic") {
    d->stateWidgetGroupBox->setTitle("Using topic");
    d->stateWidgetGroupBox->show();
  }
}

void qSlicerRos2ModuleWidget::onDescriptionSelection(const QString& text) // Shouldn't be on quit - look here: https://doc.qt.io/qt-5/qapplication.html
{
  Q_D(qSlicerRos2ModuleWidget);

  if (text == "file") {
    d->descriptionWidgetGroupBox->setTitle("File selected");
    urdfFileSelector->show();
    printButton->show();
    nodeLineEdit->hide();
    paramLineEdit->hide();
  }
  else if (text == "parameter") {
    d->descriptionWidgetGroupBox->setTitle("Param selected");
    urdfFileSelector->hide();
    printButton->hide();
    nodeLineEdit->show();
    paramLineEdit->show();
  }
}

// Slots for all of the dynamic selections start here
void qSlicerRos2ModuleWidget::onTopicNameEntered()
{
  // Get the topic name that was entered ( we will need it later)
  QString topic = topicLineEdit->text();
  std::cerr << "Topic name entered: " << topic.toStdString() << std::endl;
}

void qSlicerRos2ModuleWidget::onNodeOrParameterNameEntered(void)
{
  // Get the topic name that was entered ( we will need it later)
  QString node = nodeLineEdit->text();
  QString param = paramLineEdit->text();
  if ((!node.isEmpty())
      && (!param.isEmpty())) {
    vtkSlicerRos2Logic *
      logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
    if (!logic) {
      qWarning() << Q_FUNC_INFO << " failed: Invalid Slicer Ros2 logic";
      return;
    }
    logic->SetModelNodeAndParameter(node.toStdString(),
				    param.toStdString());
  }
}

void qSlicerRos2ModuleWidget::onDescriptionFileSelected()
{
  // Get the topic name that was entered ( we will need it later)
  QString param = paramLineEdit->text();
  std::cerr << "Param name entered: " << param.toStdString() << std::endl;
}

void qSlicerRos2ModuleWidget::onPrintButtonSelected(void)
{
  // This function lets you access the name of the urdf file that was selected in the fileDialog
  // Get the topic name that was entered ( we will need it later)
  QStringList string = urdfFileSelector->selectedFiles();
  std::string selectedFile = string.at(0).toStdString();
  vtkSlicerRos2Logic *
    logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid Slicer Ros2 logic";
    return;
  }
  logic->SetModelFile(selectedFile);
}
