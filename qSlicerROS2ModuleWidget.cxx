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
#include <QLineEdit>
#include <QUiLoader>
#include <QLineEdit>

// Slicer includes
#include "qSlicerROS2ModuleWidget.h"
#include "ui_qSlicerROS2ModuleWidget.h"
// #include "qSlicerApplication.h"
#include "ui_qSlicerROS2RobotWidget.h"

// MRML includes
#include <vtkMRMLScene.h>
#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2SubscriberNode.h>
#include <vtkMRMLROS2PublisherNode.h>
#include <vtkMRMLROS2RobotNode.h>

// Native includes
#include <iostream>
#include <filesystem>


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
}


//-----------------------------------------------------------------------------
qSlicerROS2ModuleWidget::~qSlicerROS2ModuleWidget()
{
}


//-----------------------------------------------------------------------------
void qSlicerROS2ModuleWidget::setup(void)
{
  Q_D(qSlicerROS2ModuleWidget);
  d->setupUi(this);
  this->Superclass::setup();

  // Set up signals / slots for dynamically loaded widgets
  this->connect(d->rosSubscriberTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(subscriberClicked(int, int)));
  this->connect(d->rosPublisherTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(publisherClicked(int, int)));
  this->connect(d->addNewRobotButton, SIGNAL(clicked(bool)), this, SLOT(onAddNewRobotClicked()));

  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  this->qvtkConnect(logic->mDefaultROS2Node, vtkMRMLNode::ReferenceAddedEvent,this, SLOT(updateWidget()));
  this->qvtkConnect(logic->mDefaultROS2Node, vtkMRMLNode::ReferenceRemovedEvent,this, SLOT(updateWidget()));
  updateWidget(); // if the scene is loaded before the widget is activated
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

  // update the robot widgets based on the default node robot connections
  int numRobots = logic->mDefaultROS2Node->GetNumberOfNodeReferences("robot");
  auto robotsAddedToTheNode = logic->mDefaultROS2Node->mRobotNames;
  if (robotsAddedToTheNode.size() != robotsAddedToTheWidget.size()) {
    for (int i = 0; i < numRobots; i++) {
      vtkMRMLROS2RobotNode * robot = vtkMRMLROS2RobotNode::SafeDownCast(logic->mDefaultROS2Node->GetNthNodeReference("robot", i));
      if (std::find(robotsAddedToTheWidget.begin(), robotsAddedToTheWidget.end(), robot->GetRobotName())
          != robotsAddedToTheWidget.end()) {
        continue;
      } else {
        onAddNewRobotClicked(robot->GetRobotName(), true);
        robotsAddedToTheWidget.push_back(robot->GetRobotName());
        return;
      }
    }
  }
}


void qSlicerROS2ModuleWidget::onAddNewRobotClicked(const std::string & robotName, bool active)
{
  Q_D(qSlicerROS2ModuleWidget);
  this->Superclass::setup();
  // Instantiate a robot widget
  QWidget * robotWidget = new QWidget();
  Ui_qSlicerROS2RobotWidget * robotWidgetUi = new Ui_qSlicerROS2RobotWidget();
  robotWidgetUi->setupUi(robotWidget);
  d->robotTabLayout->addWidget(robotWidget);
  auto loadRobotButton = robotWidgetUi->loadRobotButton;
  auto removeRobotButton = robotWidgetUi->removeRobotButton;
  // Set up the lambda connections
  this->connect(loadRobotButton, &QPushButton::clicked, this,
                [=]() {
                  onLoadRobotClicked(robotWidgetUi->robotNameLineEdit,
                                     robotWidgetUi->parameterNodeNameLineEdit,
                                     robotWidgetUi->parameterLineEdit,
                                     robotWidgetUi->fixedFrameLineEdit,
                                     robotWidgetUi->tfPrefixLineEdit,
                                     loadRobotButton, removeRobotButton);
                });
  this->connect(removeRobotButton, &QPushButton::clicked, this,
                [=]() {
                  onRemoveRobotClicked(robotWidgetUi->robotNameLineEdit,
                                       robotWidgetUi->parameterNodeNameLineEdit,
                                       robotWidgetUi->parameterLineEdit,
                                       robotWidgetUi->fixedFrameLineEdit,
                                       robotWidgetUi->tfPrefixLineEdit,
                                       loadRobotButton, removeRobotButton, robotWidget);
                });

  // This handles the case where a robot is added from the python console (instead of by button press)
  if (active == true) {
    robotWidgetUi->robotNameLineEdit->setEnabled(false);
    robotWidgetUi->parameterNodeNameLineEdit->setEnabled(false);
    robotWidgetUi->parameterLineEdit->setEnabled(false);
    removeRobotButton->setEnabled(true);
    QString name = QString::fromStdString(robotName);
    QLineEdit * robotNameLineEdit = robotWidgetUi->robotNameLineEdit;
    robotNameLineEdit->setText(name);
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
  // Update the publisher table widget
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


void qSlicerROS2ModuleWidget::onLoadRobotClicked(QLineEdit * robotNameLineEdit,
                                                 QLineEdit * parameterNodeNameLineEdit,
                                                 QLineEdit * parameterNameLineEdit,
                                                 QLineEdit * fixedFrameLineEdit,
                                                 QLineEdit * tfPrefixLineEdit,
                                                 QPushButton * loadRobotButton,
                                                 QPushButton * removeRobotButton)
{
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  robotsAddedToTheWidget.push_back(robotNameLineEdit->text().toStdString());
  logic->AddRobot(robotNameLineEdit->text().toStdString(),
                  parameterNodeNameLineEdit->text().toStdString(),
                  parameterNameLineEdit->text().toStdString(),
                  fixedFrameLineEdit->text().toStdString(),
                  tfPrefixLineEdit->text().toStdString());
  loadRobotButton->setEnabled(false);
  robotNameLineEdit->setEnabled(false);
  parameterNodeNameLineEdit->setEnabled(false);
  parameterNameLineEdit->setEnabled(false);
  fixedFrameLineEdit->setEnabled(false);
  tfPrefixLineEdit->setEnabled(false);
  removeRobotButton->setEnabled(true);
}


void qSlicerROS2ModuleWidget::onRemoveRobotClicked(QLineEdit * robotNameLineEdit,
                                                   QLineEdit * parameterNodeNameLineEdit,
                                                   QLineEdit * parameterNameLineEdit,
                                                   QLineEdit * fixedFrameLineEdit,
                                                   QLineEdit * tfPrefixLineEdit,
                                                   QPushButton * loadRobotButton,
                                                   QPushButton * removeRobotButton,
                                                   QWidget * robotWidget)
{
  Q_D(qSlicerROS2ModuleWidget);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  std::string robot = robotNameLineEdit->text().toStdString();
  logic->RemoveRobot(robot);
  loadRobotButton->setEnabled(true);
  robotNameLineEdit->setEnabled(true);
  parameterNodeNameLineEdit->setEnabled(true);
  parameterNameLineEdit->setEnabled(true);
  fixedFrameLineEdit->setEnabled(true);
  tfPrefixLineEdit->setEnabled(true);
  removeRobotButton->setEnabled(false);
  d->robotTabLayout->removeWidget(robotWidget);
  delete robotWidget;

  for (size_t robotName = 0; robotName < robotsAddedToTheWidget.size(); robotName++) {
    if (robotsAddedToTheWidget[robotName] == robot) {
      robotsAddedToTheWidget.erase(robotsAddedToTheWidget.begin() + robotName);
    }
  }
}
