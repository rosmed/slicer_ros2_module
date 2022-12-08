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
#include "qSlicerROS2ModuleWidget.h"
#include "ui_qSlicerROS2ModuleWidget.h"
#include "qSlicerApplication.h"

// MRML includes
#include <vtkMRMLScene.h>
#include <vtkMRMLROS2NODENode.h>
#include <vtkMRMLROS2SubscriberNode.h>
#include <vtkMRMLROS2PublisherNode.h>

// Native includes
#include <iostream>


// reference to Logic
#include "vtkSlicerROS2Logic.h"
//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerROS2ModuleWidgetPrivate: public Ui_qSlicerROS2ModuleWidget
{

public:
  qSlicerROS2ModuleWidgetPrivate();
  vtkSlicerROS2Logic* logic() const;
};


//-----------------------------------------------------------------------------
qSlicerROS2ModuleWidgetPrivate::qSlicerROS2ModuleWidgetPrivate()
{
}


//-----------------------------------------------------------------------------
// qSlicerROS2ModuleWidget methods

//-----------------------------------------------------------------------------
qSlicerROS2ModuleWidget::qSlicerROS2ModuleWidget(QWidget* _parent)
  : Superclass( _parent )
  , d_ptr( new qSlicerROS2ModuleWidgetPrivate )
{
  this->mTimer = new QTimer();
  mTimer->setSingleShot(false);
  mTimer->setInterval(10); // 20 ms, 50Hz
  mTimer->start();
}


//-----------------------------------------------------------------------------
qSlicerROS2ModuleWidget::~qSlicerROS2ModuleWidget()
{
  mTimer->stop();
  delete this->mTimer;
}


//-----------------------------------------------------------------------------
void qSlicerROS2ModuleWidget::setup(void)
{
  Q_D(qSlicerROS2ModuleWidget);
  d->setupUi(this);
  this->Superclass::setup();

  this->connect(d->clearSceneButton, SIGNAL(clicked(bool)), this, SLOT(onClearSceneSelected()));
  this->connect(d->setSubscribersButton, SIGNAL(clicked(bool)), this, SLOT(onSetSubscribers()));
  this->connect(d->setPublishersButton, SIGNAL(clicked(bool)), this, SLOT(onSetPublishers()));
  this->connect(d->addNodeButton, SIGNAL(clicked(bool)), this, SLOT(onNodeAddedButton()));

  // Set up timer connections
  connect(mTimer, SIGNAL(timeout()), this, SLOT(onTimerTimeOut()));
  connect(qSlicerApplication::application(), SIGNAL(lastWindowClosed()), this, SLOT(stopTimer()));

  // Set up signals / slots for dynamically loaded widgets
  this->connect(d->loadVisualizationButton, SIGNAL(clicked(bool)), this, SLOT(onNodeOrParameterNameEntered()));
  this->connect(d->rosSubscriberTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(subscriberClicked(int, int)));
  this->connect(d->rosPublisherTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(publisherClicked(int, int)));
  this->connect(d->broadcastTransformButton, SIGNAL(clicked(bool)), this, SLOT(onBroadcastButtonPressed()));

}

void qSlicerROS2ModuleWidget::onNodeAddedButton()
{
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  // Instantiate ROS2 node
  logic->AddROS2Node();

  // Setup modified connections for the widget
  if (logic->mROS2Node){
    qvtkReconnect(logic->mROS2Node, vtkMRMLNode::ReferenceAddedEvent, this, SLOT(updateWidget())); // Set up observer
  }
}



void qSlicerROS2ModuleWidget::onTimerTimeOut()
{
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->Spin();
}

void qSlicerROS2ModuleWidget::updateWidget()
{
  Q_D(qSlicerROS2ModuleWidget);
  this->Superclass::setup();
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
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
  // update publisher table
  if (visiblePublisherRefs < publisherRefs){
   refreshPubTable();
  }
}

void qSlicerROS2ModuleWidget::refreshSubTable()
{
  Q_D(qSlicerROS2ModuleWidget);
  this->Superclass::setup();
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  // Update the subscriber table widget
  size_t subRow = 0;
  d->rosSubscriberTableWidget->clearContents();

  // Resize the table
  d->rosSubscriberTableWidget->setRowCount(logic->mROS2Node->GetNumberOfNodeReferences("subscriber"));

  // Iterate through the references
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

void qSlicerROS2ModuleWidget::refreshPubTable()
{
  Q_D(qSlicerROS2ModuleWidget);
  this->Superclass::setup();
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  // Update the publisher table wideget
  size_t pubRow = 0;
  d->rosPublisherTableWidget->clearContents();

  // Resize the table
  d->rosPublisherTableWidget->setRowCount(logic->mROS2Node->GetNumberOfNodeReferences("publisher"));

  // Iterate through the references
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

void qSlicerROS2ModuleWidget::updateSubscriberTable(vtkMRMLROS2SubscriberNode* sub, size_t row){
  Q_D(qSlicerROS2ModuleWidget);
  this->Superclass::setup();
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }

  // Get the subscriber topic and message type and populate the cells in the table with this info
  QString topicName = sub->GetTopic();
  QString typeName = sub->GetROSType();

  QTableWidgetItem *topic_item = d->rosSubscriberTableWidget->item(row, 0);
  QTableWidgetItem *type_item = d->rosSubscriberTableWidget->item(row, 1);

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

void qSlicerROS2ModuleWidget::updatePublisherTable(vtkMRMLROS2PublisherNode* sub, size_t row){
  Q_D(qSlicerROS2ModuleWidget);
  this->Superclass::setup();
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }

  // Get the subscriber topic and message type and populate the cells in the table with this info
  QString topicName = sub->GetTopic();
  QString typeName = sub->GetROSType();

  QTableWidgetItem *topic_item = d->rosPublisherTableWidget->item(row, 0);
  QTableWidgetItem *type_item = d->rosPublisherTableWidget->item(row, 2);

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

void qSlicerROS2ModuleWidget::onClearSceneSelected()
{
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->Clear();
}



void qSlicerROS2ModuleWidget::onSetSubscribers()
{
  Q_D(qSlicerROS2ModuleWidget);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  if (logic->mROS2Node){
    logic->AddToScene();
    refreshSubTable();
  }
  else{
    QLabel *popupLabel = new QLabel();
    QString message = "ROS2 node has not been added yet.";
    popupLabel->setText(message);
    popupLabel->show();
  }
}

void qSlicerROS2ModuleWidget::onSetPublishers()
{
  Q_D(qSlicerROS2ModuleWidget);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  if (logic->mROS2Node){
    logic->AddPublisher();
    refreshPubTable();
  }
  else{
    QLabel *popupLabel = new QLabel();
    QString message = "ROS2 node has not been added yet.";
    popupLabel->setText(message);
    popupLabel->show();
  }
}

void qSlicerROS2ModuleWidget::subscriberClicked(int row, int col)
{
  // Row is a reference to the message index
  Q_D(qSlicerROS2ModuleWidget);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  if (col == 1){ // only invoked when users click the number of messages cell
    QString subName = d->rosSubscriberTableWidget->item(row,0)->text();
    std::string topic = subName.toStdString();
    vtkMRMLROS2SubscriberNode *sub = vtkMRMLROS2SubscriberNode::SafeDownCast(logic->GetMRMLScene()->GetFirstNodeByName(("ros2:sub:" + topic).c_str()));
    if (!sub){
      std::cerr << "No subscriber by this name in the scene" << std::endl;
      return;
    }
    QString message = sub->GetLastMessageYAML().c_str();
    QString numMessages = QVariant(static_cast<int>(sub->GetNumberOfMessages())).toString(); // to convert to an int and then string
    QLabel *popupLabel = new QLabel();
    QString numMess = "Num messages:   ";
    QString mess = "   Message: ";
    popupLabel->setText(numMess + numMessages + mess + message);
    popupLabel->show();
  }
}

void qSlicerROS2ModuleWidget::publisherClicked(int row, int col)
{
  // Row is a reference to the message index
  Q_D(qSlicerROS2ModuleWidget);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  if (col == 1){ // only invoked when users click the number of messages cell
    QString pubName = d->rosPublisherTableWidget->item(row,0)->text();
    std::string topic = pubName.toStdString();
    vtkMRMLROS2PublisherNode *pub = vtkMRMLROS2PublisherNode::SafeDownCast(logic->GetMRMLScene()->GetFirstNodeByName(("ros2:pub:" + topic).c_str()));
    if (!pub){
      std::cerr << "No publisher by this name in the scene" << std::endl;
      return;
    }
    QString numMessages = QVariant(static_cast<int>(pub->GetNumberOfCalls())).toString(); // to convert to an int and then string
    std::cerr << pub->GetNumberOfCalls() << std::endl;
    QLabel *popupLabel = new QLabel();
    QString numMess = "Num of calls:   ";
    QString numSent = "     Num of messages sent: ";
    QString numMessagesSent = QVariant(static_cast<int>(pub->GetNumberOfMessagesSent(logic->mROS2Node->GetID(), topic))).toString();
    popupLabel->setText(numMess + numMessages + numSent + numMessagesSent);
    popupLabel->show();
  }
}

void qSlicerROS2ModuleWidget::stopTimer(void) // Shouldn't be on quit - look here: https://doc.qt.io/qt-5/qapplication.html
{
  mTimer->stop();
}


void qSlicerROS2ModuleWidget::onStateSelection(const QString& text)
{
  Q_D(qSlicerROS2ModuleWidget);
  // Note: this logic part is repeated a lot there is probably a way to avoid that
  vtkSlicerROS2Logic *
    logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  if (text == "tf2") {
    logic->SetRobotStateTf();
  }
}


void qSlicerROS2ModuleWidget::onDescriptionSelection(const QString& text) // Shouldn't be on quit - look here: https://doc.qt.io/qt-5/qapplication.html
{
  Q_D(qSlicerROS2ModuleWidget);

  if (text == "parameter") {
    d->descriptionWidgetGroupBox->setTitle("Param selected");
    d->nodeLineEdit->show();
    d->paramLineEdit->show();
  }
}


void qSlicerROS2ModuleWidget::onNodeOrParameterNameEntered(void)
{
  // Get the parameter and node names (we will need it later)
  Q_D(qSlicerROS2ModuleWidget);
  QString node = d->nodeLineEdit->text();
  QString param = d->paramLineEdit->text();
  if ((!node.isEmpty())
      && (!param.isEmpty())) {
    vtkSlicerROS2Logic *
      logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
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


void qSlicerROS2ModuleWidget::onLoadModelButtonSelected(void)
{
  // This function lets you access the name of the urdf file that was selected in the fileDialog
  // Get the topic name that was entered ( we will need it later)
  QStringList files = urdfFileSelector->selectedFiles();
  if (files.isEmpty()) {
    qWarning() << "SlicerROS2: no file selected";
    return;
  }
  std::string selectedFile = files.at(0).toStdString();
  vtkSlicerROS2Logic *
    logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->SetModelFile(selectedFile);
}


void qSlicerROS2ModuleWidget::onBroadcastButtonPressed()
{
  vtkSlicerROS2Logic *
    logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->BroadcastTransform();
}
