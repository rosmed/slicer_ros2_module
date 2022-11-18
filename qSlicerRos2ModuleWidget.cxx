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
#include <QTableWidgetItem>
#include <QString>

// Slicer includes
#include "qSlicerRos2ModuleWidget.h"
#include "ui_qSlicerRos2ModuleWidget.h"
#include "qSlicerApplication.h"

// reference to Logic
#include "vtkSlicerRos2Logic.h"

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
  mTimer->setInterval(10); // 20 ms, 50Hz
  mTimer->start();
}


//-----------------------------------------------------------------------------
qSlicerRos2ModuleWidget::~qSlicerRos2ModuleWidget()
{
  mTimer->stop();
  delete this->mTimer;
}


//-----------------------------------------------------------------------------
void qSlicerRos2ModuleWidget::setup(void)
{
  Q_D(qSlicerRos2ModuleWidget);
  d->setupUi(this);
  this->Superclass::setup();

  this->connect(d->clearSceneButton, SIGNAL(clicked(bool)), this, SLOT(onClearSceneSelected()));
  this->connect(d->setSubscribersButton, SIGNAL(clicked(bool)), this, SLOT(onSetSubscribers()));

  // Set up timer connections
  connect(mTimer, SIGNAL(timeout()), this, SLOT(onTimerTimeOut()));
  connect(qSlicerApplication::application(), SIGNAL(lastWindowClosed()), this, SLOT(stopTimer()));

  // Setup state / selection options
  this->connect(d->stateSelectionComboBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(onStateSelection(const QString&)));

  this->connect(d->descriptionSelectionComboBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(onDescriptionSelection(const QString&)));

  // Set up signals / slots for dynamically loaded widgets
  // Note: All of the QLineEdits are triggered by pressing enter in the edit box - the slot functions access the text that was entered themselves
  this->connect(d->topicLineEdit, SIGNAL(returnPressed()), this, SLOT(onTopicNameEntered()));
  this->connect(d->nodeLineEdit, SIGNAL(returnPressed()), this, SLOT(onNodeOrParameterNameEntered()));
  this->connect(d->paramLineEdit, SIGNAL(returnPressed()), this, SLOT(onNodeOrParameterNameEntered()));
  this->connect(d->selectFileButton, SIGNAL(clicked(bool)), this, SLOT(onSelectFile()));
  this->connect(d->loadModelButton, SIGNAL(clicked(bool)), this, SLOT(onLoadModelButtonSelected()));
  // file dialog signals are weird so using the button as a place holder just so you can print the name of the file you selected

  // Set default, assuming defaults are:
  // - state if from tf
  // - model is from param
  d->stateWidgetGroupBox->hide();
  d->loadModelButton->hide();
  d->selectFileButton->hide();
  d->rosSubscriberTableWidget->resizeColumnsToContents();

  this->connect(d->broadcastTransformButton, SIGNAL(clicked(bool)), this, SLOT(onBroadcastButtonPressed()));

}


void qSlicerRos2ModuleWidget::onFileSelected(const QString&)
{
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }

  // Check if the timer is on or off before setting up the robot
  // Anton: is this still needed?
  if (timerOff == true) {
    mTimer->start();
    timerOff = false;
  }
}


void qSlicerRos2ModuleWidget::onTimerTimeOut()
{
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->Spin();
  updateSubscriberTableWidget();
}

void qSlicerRos2ModuleWidget::updateSubscriberTableWidget()
{
  Q_D(qSlicerRos2ModuleWidget);
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }

  d->rosSubscriberTableWidget->setRowCount(logic->mSubs.size());
  size_t row = 0;
  for (const auto sub : logic->mSubs) {
    QString topicName = sub->GetTopic();
    QString typeName = sub->GetROSType();
    QString message = sub->GetLastMessageYAML().c_str();
    QTableWidgetItem *topic_item = d->rosSubscriberTableWidget->item(row, 0);
    QTableWidgetItem *type_item = d->rosSubscriberTableWidget->item(row, 1);
    QTableWidgetItem *message_item = d->rosSubscriberTableWidget->item(row, 2);
    // if the row doesn't exist, populate
    if (!topic_item) {
      topic_item = new QTableWidgetItem;
      d->rosSubscriberTableWidget->setItem(row, 0, topic_item);
      topic_item->setText(topicName);
      type_item = new QTableWidgetItem;
      d->rosSubscriberTableWidget->setItem(row, 1, type_item);
      type_item->setText(typeName);
      message_item = new QTableWidgetItem;
      d->rosSubscriberTableWidget->setItem(row, 2, message_item);
    }
    // otherwise, just update the message
    else {
      message_item->setText(message);
    }
    row++;
  }
}

void qSlicerRos2ModuleWidget::onClearSceneSelected()
{
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->Clear();
}

void qSlicerRos2ModuleWidget::onSetSubscribers()
{
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->AddToScene();
}

void qSlicerRos2ModuleWidget::stopTimer(void) // Shouldn't be on quit - look here: https://doc.qt.io/qt-5/qapplication.html
{
  mTimer->stop();
}


void qSlicerRos2ModuleWidget::onStateSelection(const QString& text)
{
  Q_D(qSlicerRos2ModuleWidget);
  // Note: this logic part is repeated a lot there is probably a way to avoid that
  vtkSlicerRos2Logic *
    logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  if (text == "tf2") {
    d->stateWidgetGroupBox->hide();
    logic->SetRobotStateTf();
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
    d->loadModelButton->show();
    d->selectFileButton->show();
    d->nodeLineEdit->hide();
    d->paramLineEdit->hide();
  }
  else if (text == "parameter") {
    d->descriptionWidgetGroupBox->setTitle("Param selected");
    d->loadModelButton->hide();
    d->selectFileButton->hide();
    d->nodeLineEdit->show();
    d->paramLineEdit->show();
  }
}


// Slots for all of the dynamic selections start here
void qSlicerRos2ModuleWidget::onTopicNameEntered(void)
{
  Q_D(qSlicerRos2ModuleWidget);
  // Get the topic name (we will need it later)
  QString topic = d->topicLineEdit->text();
  vtkSlicerRos2Logic *
    logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->SetRobotStateTopic(topic.toStdString());
  qDebug() << "SlicerROS2: using topic " << topic;
}


void qSlicerRos2ModuleWidget::onNodeOrParameterNameEntered(void)
{
  // Get the parameter and node names (we will need it later)
  Q_D(qSlicerRos2ModuleWidget);
  QString node = d->nodeLineEdit->text();
  QString param = d->paramLineEdit->text();
  if ((!node.isEmpty())
      && (!param.isEmpty())) {
    vtkSlicerRos2Logic *
      logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
    if (!logic) {
      qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
      return;
    }
    logic->SetModelNodeAndParameter(node.toStdString(),
				    param.toStdString());
    qDebug() << "SlicerROS2: using parameter " << param
	     << " from node " << node;
  }
}


void qSlicerRos2ModuleWidget::onSelectFile(void)
{
  urdfFileSelector->show();
}


void qSlicerRos2ModuleWidget::onLoadModelButtonSelected(void)
{
  // This function lets you access the name of the urdf file that was selected in the fileDialog
  // Get the topic name that was entered ( we will need it later)
  QStringList files = urdfFileSelector->selectedFiles();
  if (files.isEmpty()) {
    qWarning() << "SlicerROS2: no file selected";
    return;
  }
  std::string selectedFile = files.at(0).toStdString();
  vtkSlicerRos2Logic *
    logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->SetModelFile(selectedFile);
}


void qSlicerRos2ModuleWidget::onBroadcastButtonPressed()
{
  vtkSlicerRos2Logic *
    logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->BroadcastTransform();
}
