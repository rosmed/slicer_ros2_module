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
#include <QVariant>
#include <QPushButton>
#include <QLabel>

// Slicer includes
#include "qSlicerRos2ModuleWidget.h"
#include "ui_qSlicerRos2ModuleWidget.h"
#include "qSlicerApplication.h"

#include <vtkMRMLScene.h>
#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2SubscriberNode.h>
#include <vtkMRMLROS2PublisherNode.h>


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
  this->connect(d->setPublishersButton, SIGNAL(clicked(bool)), this, SLOT(onSetPublishers()));
  this->connect(d->addNodeButton, SIGNAL(clicked(bool)), this, SLOT(onNodeAddedButton()));

  // Set up timer connections
  connect(mTimer, SIGNAL(timeout()), this, SLOT(onTimerTimeOut()));
  connect(qSlicerApplication::application(), SIGNAL(lastWindowClosed()), this, SLOT(stopTimer()));

  // Setup state / selection options
  // this->connect(d->stateSelectionComboBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(onStateSelection(const QString&)));

  // this->connect(d->descriptionSelectionComboBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(onDescriptionSelection(const QString&)));

  // Set up signals / slots for dynamically loaded widgets
  // Note: All of the QLineEdits are triggered by pressing enter in the edit box - the slot functions access the text that was entered themselves
  this->connect(d->loadVisualizationButton, SIGNAL(clicked(bool)), this, SLOT(onNodeOrParameterNameEntered()));
  this->connect(d->rosSubscriberTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(subscriberClicked(int, int)));
  // this->connect(d->rosPublisherTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(publisherClicked(int, int)));
  // file dialog signals are weird so using the button as a place holder just so you can print the name of the file you selected

  // Set default, assuming defaults are:
  // - state if from tf
  // - model is from param
  // d->stateWidgetGroupBox->hide();
  // d->loadModelButton->hide();
  // d->selectFileButton->hide();
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

void qSlicerRos2ModuleWidget::onNodeAddedButton()
{
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->AddROS2Node();
  // Setup modified connections for the widget
  if (logic->mROS2Node){
    std::cerr << "Event connected" << std::endl;
    qvtkReconnect(logic->mROS2Node, vtkMRMLNode::ReferenceAddedEvent, this, SLOT(updateWidget())); // Set up observer
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
}


void qSlicerRos2ModuleWidget::refreshSubTable()
{
  Q_D(qSlicerRos2ModuleWidget);
  this->Superclass::setup();
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  size_t subRow = 0;
  d->rosSubscriberTableWidget->clearContents();
  d->rosSubscriberTableWidget->setRowCount(logic->mROS2Node->GetNumberOfNodeReferences("subscriber"));
  for (int index = 0; index < logic->mROS2Node->GetNumberOfNodeReferences("subscriber"); ++index) {
  const char * id = logic->mROS2Node->GetNthNodeReferenceID("subscriber", index);
  vtkMRMLROS2SubscriberNode *sub = vtkMRMLROS2SubscriberNode::SafeDownCast(logic->mROS2Node->GetScene()->GetNodeByID(id));
  if (sub == nullptr) {
    } else {
      updateSubscriberTable(sub, subRow);
      subRow++;
    }
  }
}

void qSlicerRos2ModuleWidget::refreshPubTable()
{
  Q_D(qSlicerRos2ModuleWidget);
  this->Superclass::setup();
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  size_t pubRow = 0;
  d->rosPublisherTableWidget->clearContents();
  d->rosPublisherTableWidget->setRowCount(logic->mROS2Node->GetNumberOfNodeReferences("publisher"));
  for (int index = 0; index < logic->mROS2Node->GetNumberOfNodeReferences("publisher"); ++index) {
    const char * id = logic->mROS2Node->GetNthNodeReferenceID("publisher", index);
    vtkMRMLROS2PublisherNode *pub = vtkMRMLROS2PublisherNode::SafeDownCast(logic->mROS2Node->GetScene()->GetNodeByID(id));
    if (pub == nullptr) {
    } else {
      updatePublisherTable(pub, pubRow);
      pubRow++;
    }
  }
}

void qSlicerRos2ModuleWidget::updateWidget()
{
  Q_D(qSlicerRos2ModuleWidget);
  this->Superclass::setup();
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }

  // Check how many subscriber references are on the node vs. how many rows are in the table
  int visibleSubscriberRefs = d->rosSubscriberTableWidget->rowCount();
  int visiblePublisherRefs = d->rosPublisherTableWidget->rowCount();

  int subscriberRefs = logic->mROS2Node->GetNumberOfNodeReferences("subscriber");
  int publisherRefs = logic->mROS2Node->GetNumberOfNodeReferences("publisher");

  // update subscriber table 
  if (visibleSubscriberRefs < subscriberRefs){
    refreshSubTable();
  }

  if (visiblePublisherRefs < publisherRefs){
   refreshPubTable();
  }
}

void qSlicerRos2ModuleWidget::updateSubscriberTable(vtkMRMLROS2SubscriberNode* sub, size_t row){
  Q_D(qSlicerRos2ModuleWidget);
  this->Superclass::setup();
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }

  QString topicName = sub->GetTopic();
  QString typeName = sub->GetROSType();
  
  QTableWidgetItem *topic_item = d->rosSubscriberTableWidget->item(row, 0);
  QTableWidgetItem *type_item = d->rosSubscriberTableWidget->item(row, 1);

  // if the row doesn't exist, populate
  if (!topic_item) {
    topic_item = new QTableWidgetItem;
    d->rosSubscriberTableWidget->setItem(row, 0, topic_item);
    topic_item->setText(topicName);
    type_item = new QTableWidgetItem;
    d->rosSubscriberTableWidget->setItem(row, 1, type_item);
    type_item->setText(typeName);
    std::cerr << "Type name:" << typeName.toStdString() << std::endl;
  }
  row++;
}

void qSlicerRos2ModuleWidget::updatePublisherTable(vtkMRMLROS2PublisherNode* sub, size_t row){
  Q_D(qSlicerRos2ModuleWidget);
  this->Superclass::setup();
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }


  QString topicName = sub->GetTopic();
  QString typeName = sub->GetROSType();
  QTableWidgetItem *topic_item = d->rosPublisherTableWidget->item(row, 0);
  QTableWidgetItem *type_item = d->rosPublisherTableWidget->item(row, 2);

  // if the row doesn't exist, populate
  if (!topic_item) {
    topic_item = new QTableWidgetItem;
    d->rosPublisherTableWidget->setItem(row, 0, topic_item);
    topic_item->setText(topicName);
    type_item = new QTableWidgetItem;
    d->rosPublisherTableWidget->setItem(row, 1, type_item);
    type_item->setText(typeName);
  }
  row++;
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
  Q_D(qSlicerRos2ModuleWidget);
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->AddToScene();
  refreshSubTable();
}

void qSlicerRos2ModuleWidget::onSetPublishers()
{
  Q_D(qSlicerRos2ModuleWidget);
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->AddPublisher();
  refreshPubTable();
}

void qSlicerRos2ModuleWidget::subscriberClicked(int row, int col)
{
  // Row is a reference to the message index
  Q_D(qSlicerRos2ModuleWidget);
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  if (col == 1){ // only invoked when users click the number of messages cell
    QString subName = d->rosSubscriberTableWidget->item(row,0)->text();
    std::string referenceRole = subName.toStdString();
    vtkMRMLROS2SubscriberNode *sub = vtkMRMLROS2SubscriberNode::SafeDownCast(logic->GetMRMLScene()->GetFirstNodeByName(("ros2:sub:" + referenceRole).c_str()));
    QString message = sub->GetLastMessageYAML().c_str();
    QString numMessages = QVariant(static_cast<int>(sub->GetNumberOfMessages())).toString(); // to convert to an int and then string
    QLabel *popupLabel = new QLabel();
    QString numMess = "Num messages:   ";
    QString mess = "   Message: ";
    popupLabel->setText(numMess + numMessages + mess + message);
    popupLabel->show();
  }
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
    logic->SetRobotStateTf();
  }
}


void qSlicerRos2ModuleWidget::onDescriptionSelection(const QString& text) // Shouldn't be on quit - look here: https://doc.qt.io/qt-5/qapplication.html
{
  Q_D(qSlicerRos2ModuleWidget);

  if (text == "parameter") {
    d->descriptionWidgetGroupBox->setTitle("Param selected");
    d->nodeLineEdit->show();
    d->paramLineEdit->show();
  }
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
    logic->SetRobotStateTf();
    logic->SetModelNodeAndParameter(node.toStdString(),
				    param.toStdString());
    qDebug() << "SlicerROS2: using parameter " << param
	     << " from node " << node;
  }
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
