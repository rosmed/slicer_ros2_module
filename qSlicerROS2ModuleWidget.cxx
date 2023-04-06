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
#include <QMessageBox>

// Slicer includes
#include "qSlicerROS2ModuleWidget.h"
#include "ui_qSlicerROS2ModuleWidget.h"
#include "qSlicerApplication.h"

// MRML includes
#include <vtkMRMLScene.h>
#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2SubscriberNode.h>
#include <vtkMRMLROS2PublisherNode.h>
#include <vtkMRMLROS2RobotNode.h>

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
  mTimer->setInterval(20); // 20 ms, 50Hz
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

  // Set up timer connections
  connect(mTimer, SIGNAL(timeout()), this, SLOT(onTimerTimeOut()));
  connect(qSlicerApplication::application(), SIGNAL(lastWindowClosed()), this, SLOT(stopTimer()));

  // Set up signals / slots for dynamically loaded widgets
  this->connect(d->rosSubscriberTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(subscriberClicked(int, int)));
  this->connect(d->rosPublisherTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(publisherClicked(int, int)));

  // Robot setup
  this->connect(d->loadRobotButton, SIGNAL(clicked(bool)), this, SLOT(onLoadRobotClicked()));
  this->connect(d->loadRobot2Button, SIGNAL(clicked(bool)), this, SLOT(onLoadRobot2Clicked()));
  this->connect(d->removeRobotButton, SIGNAL(clicked(bool)), this, SLOT(onRemoveRobotClicked()));
  this->connect(d->removeRobot2Button, SIGNAL(clicked(bool)), this, SLOT(onRemoveRobot2Clicked()));
  d->removeRobotButton->setEnabled(false);

  // Robot 2 control
  this->connect(d->addRobotButton, SIGNAL(clicked(bool)), this, SLOT(onAddRobotButton()));

  d->loadRobot2Button->hide();
  d->removeRobot2Button->hide();
  d->robot2NameLineEdit->hide();
  d->parameterNode2NameLineEdit->hide();
  d->parameter2LineEdit->hide();
  d->robot2NameLabel->hide();
  d->parameterNode2Label->hide();
  d->parameter2Label->hide();

  this->Superclass::setup();
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  this->qvtkConnect(logic->mDefaultROS2Node, vtkMRMLNode::ReferenceAddedEvent,this, SLOT(updateWidget()));
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

  // The following needs to be updated to list all ROS nodes in logic 
  int subscriberRefs = logic->mDefaultROS2Node->GetNumberOfNodeReferences("subscriber");
  int publisherRefs = logic->mDefaultROS2Node->GetNumberOfNodeReferences("publisher");

  // update subscriber table
  if (visibleSubscriberRefs < subscriberRefs) {
    refreshSubTable();
  }
  // update publisher table
  if (visiblePublisherRefs < publisherRefs) {
    refreshPubTable();
  }

  // Update robot selector
  vtkMRMLROS2RobotNode * robotNode = dynamic_cast<vtkMRMLROS2RobotNode *>(logic->GetMRMLScene()->GetFirstNodeByClass("vtkMRMLROS2RobotNode"));
  if ( robotNode != NULL )
  {
    QString robotName = QString::fromStdString(robotNode->GetRobotName());
    d->robotNameLineEdit->setText(robotName);
    this->onLoadRobotClicked();
    return;
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

  // // Resize the table
  d->rosSubscriberTableWidget->setRowCount(logic->mDefaultROS2Node->GetNumberOfNodeReferences("subscriber"));

  // // Iterate through the references
  for (int index = 0; index < logic->mDefaultROS2Node->GetNumberOfNodeReferences("subscriber"); ++index) {
  const char * id = logic->mDefaultROS2Node->GetNthNodeReferenceID("subscriber", index);
  vtkMRMLROS2SubscriberNode *sub = vtkMRMLROS2SubscriberNode::SafeDownCast(logic->mDefaultROS2Node->GetScene()->GetNodeByID(id));
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
  d->rosPublisherTableWidget->setRowCount(logic->mDefaultROS2Node->GetNumberOfNodeReferences("publisher"));

  // Iterate through the references
  for (int index = 0; index < logic->mDefaultROS2Node->GetNumberOfNodeReferences("publisher"); ++index) {
    const char * id = logic->mDefaultROS2Node->GetNthNodeReferenceID("publisher", index);
    vtkMRMLROS2PublisherNode *pub = vtkMRMLROS2PublisherNode::SafeDownCast(logic->mDefaultROS2Node->GetScene()->GetNodeByID(id));
    if (pub == nullptr) {
    } else {
      updatePublisherTable(pub, pubRow);
      pubRow++;
    }
  }
}


void qSlicerROS2ModuleWidget::updateSubscriberTable(vtkMRMLROS2SubscriberNode* sub, size_t row)
{
  Q_D(qSlicerROS2ModuleWidget);
  this->Superclass::setup();
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }

  QTableWidgetItem *topic_item = d->rosSubscriberTableWidget->item(row, 0);
  QTableWidgetItem *type_item = d->rosSubscriberTableWidget->item(row, 1);

  if (!topic_item) {
    topic_item = new QTableWidgetItem;
    d->rosSubscriberTableWidget->setItem(row, 0, topic_item);
    topic_item->setText(sub->GetTopic().c_str());
    type_item = new QTableWidgetItem;
    d->rosSubscriberTableWidget->setItem(row, 1, type_item);
    type_item->setText(sub->GetROSType());
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

  QTableWidgetItem *topic_item = d->rosPublisherTableWidget->item(row, 0);
  QTableWidgetItem *type_item = d->rosPublisherTableWidget->item(row, 2);

  if (!topic_item) {
    topic_item = new QTableWidgetItem;
    d->rosPublisherTableWidget->setItem(row, 0, topic_item);
    topic_item->setText(sub->GetTopic().c_str());
    type_item = new QTableWidgetItem;
    d->rosPublisherTableWidget->setItem(row, 1, type_item);
    type_item->setText(sub->GetROSType());
  }
  row++;
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
  if (col == 1) { // only invoked when users click the number of messages cell
    QString subName = d->rosSubscriberTableWidget->item(row,0)->text();
    std::string topic = subName.toStdString();
    vtkMRMLROS2SubscriberNode *sub = vtkMRMLROS2SubscriberNode::SafeDownCast(logic->GetMRMLScene()->GetFirstNodeByName(("ros2:sub:" + topic).c_str()));
    if (!sub) {
      std::cerr << "No subscriber by this name in the scene" << std::endl;
      return;
    }
    QMessageBox msgBox;
    msgBox.setText(QStringLiteral("Number of messages: %1\nLast message: %2")
		   .arg(sub->GetNumberOfMessages())
		   .arg(sub->GetLastMessageYAML().c_str()));
    msgBox.exec();
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
  if (col == 1) { // only invoked when users click the number of messages cell
    QString pubName = d->rosPublisherTableWidget->item(row,0)->text();
    std::string topic = pubName.toStdString();
    vtkMRMLROS2PublisherNode *pub = vtkMRMLROS2PublisherNode::SafeDownCast(logic->GetMRMLScene()->GetFirstNodeByName(("ros2:pub:" + topic).c_str()));
    if (!pub) {
      std::cerr << "No publisher by this name in the scene" << std::endl;
      return;
    }
    QMessageBox msgBox;
    msgBox.setText(QStringLiteral("Number of calls: %1\nNumber of messages sent: %2")
		   .arg(pub->GetNumberOfCalls())
		   .arg(pub->GetNumberOfMessagesSent()));
    msgBox.exec();
  }
}


void qSlicerROS2ModuleWidget::stopTimer(void) // Shouldn't be on quit - look here: https://doc.qt.io/qt-5/qapplication.html
{
  mTimer->stop();
}


void qSlicerROS2ModuleWidget::onLoadRobotClicked(void)
{
  Q_D(qSlicerROS2ModuleWidget);
  std::string robotName = d->robotNameLineEdit->text().toStdString();
  std::string parameterNodeName = d->parameterNodeNameLineEdit->text().toStdString();
  std::string parameterName = d->parameterLineEdit->text().toStdString();
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->AddRobot(parameterNodeName, parameterName, robotName);

  // Disable the UI
  d->removeRobotButton->setEnabled(true);
  d->loadRobotButton->setEnabled(false);
  d->robotNameLineEdit->setEnabled(false);
  d->parameterNodeNameLineEdit->setEnabled(false);
  d->parameterLineEdit->setEnabled(false);
}

void qSlicerROS2ModuleWidget::onLoadRobot2Clicked(void)
{
  Q_D(qSlicerROS2ModuleWidget);
  std::string robotName = d->robot2NameLineEdit->text().toStdString();
  std::string parameterNodeName = d->parameterNode2NameLineEdit->text().toStdString();
  std::string parameterName = d->parameter2LineEdit->text().toStdString();
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->AddRobot(parameterNodeName, parameterName, robotName);

  // Disable the UI
  d->removeRobot2Button->setEnabled(true);
  d->loadRobot2Button->setEnabled(false);
  d->robot2NameLineEdit->setEnabled(false);
  d->parameterNode2NameLineEdit->setEnabled(false);
  d->parameter2LineEdit->setEnabled(false);
}

void qSlicerROS2ModuleWidget::onRemoveRobotClicked(void)
{
  Q_D(qSlicerROS2ModuleWidget);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->RemoveRobot(d->robotNameLineEdit->text().toStdString());
  d->removeRobotButton->setEnabled(false);
  d->loadRobotButton->setEnabled(true);
  d->robotNameLineEdit->setEnabled(true);
  d->parameterNodeNameLineEdit->setEnabled(true);
  d->parameterLineEdit->setEnabled(true);
}

void qSlicerROS2ModuleWidget::onRemoveRobot2Clicked(void)
{
  Q_D(qSlicerROS2ModuleWidget);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  logic->RemoveRobot(d->robot2NameLineEdit->text().toStdString());
  d->removeRobot2Button->setEnabled(false);
  d->loadRobot2Button->setEnabled(true);
  d->robot2NameLineEdit->setEnabled(true);
  d->parameterNode2NameLineEdit->setEnabled(true);
  d->parameter2LineEdit->setEnabled(true);
}

void qSlicerROS2ModuleWidget::onAddRobotButton(void)
{
  Q_D(qSlicerROS2ModuleWidget);
  d->addRobotButton->hide();
  d->loadRobot2Button->show();
  d->removeRobot2Button->show();
  d->robot2NameLineEdit->show();
  d->parameterNode2NameLineEdit->show();
  d->parameter2LineEdit->show();
  d->robot2NameLabel->show();
  d->parameterNode2Label->show();
  d->parameter2Label->show();
}
