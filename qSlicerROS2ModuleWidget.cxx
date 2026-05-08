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
#include <QCheckBox> 
#include <QTimer>
#include <QMenu>
#include <QAction>
#include <cmath>

// Slicer includes
#include "qSlicerROS2ModuleWidget.h"
#include "ui_qSlicerROS2ModuleWidget.h"
#include "ui_qSlicerROS2RobotWidget.h"

// MRML includes
#include <vtkMRMLScene.h>
#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2SubscriberNode.h>
#include <vtkMRMLROS2PublisherNode.h>
#include <vtkMRMLROS2RobotNode.h>
#include <vtkMRMLROS2ParameterNode.h>
#include <vtkMRMLROS2Tf2BroadcasterNode.h>
#include <vtkMRMLROS2Tf2LookupNode.h>
#include <vtkMRMLModelNode.h>
#include <vtkMRMLModelDisplayNode.h>
#include <vtkMRMLLinearTransformNode.h>

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
  observeSubscriber(nullptr);
  observeParameter(nullptr);
  observeTf2Lookup(nullptr);
}


//-----------------------------------------------------------------------------
void qSlicerROS2ModuleWidget::setup(void)
{
  Q_D(qSlicerROS2ModuleWidget);
  d->setupUi(this);
  this->Superclass::setup();

  // Set up signals / slots for dynamically loaded widgets
  this->connect(d->addNewRobotButton, SIGNAL(clicked(bool)), this, SLOT(onAddNewRobotClicked()));

  // Topics
  this->connect(d->rosSubscriberTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(subscriberRowSelected(int, int)));
  this->connect(d->rosPublisherTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(publisherRowSelected(int, int)));

  // Parameters
  this->connect(d->parameterNodeTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(parameterNodeRowSelected(int, int)));

  // Tf2
  this->connect(d->tf2LookupTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(tf2LookupRowSelected(int, int)));
  
  // Display mode menu for Tf2
  QMenu* tf2Menu = new QMenu(this);
  QAction* actionMatrix = tf2Menu->addAction("Matrix 4x4");
  QAction* actionRPY = tf2Menu->addAction("RPY + XYZ");
  QAction* actionQuat = tf2Menu->addAction("Unit Quaternion");
  
  connect(actionMatrix, &QAction::triggered, this, [this](){ setTf2DisplayMode(0); });
  connect(actionRPY, &QAction::triggered, this, [this](){ setTf2DisplayMode(1); });
  connect(actionQuat, &QAction::triggered, this, [this](){ setTf2DisplayMode(2); });
  d->tf2DisplayModeButton->setMenu(tf2Menu);

  // Tab changes
  this->connect(d->tabWidget_2, SIGNAL(currentChanged(int)), this, SLOT(onInnerTabChanged(int)));

  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  this->qvtkConnect(logic->mDefaultROS2Node, vtkMRMLNode::ReferenceAddedEvent,this, SLOT(updateWidget()));
  this->qvtkConnect(logic->mDefaultROS2Node, vtkMRMLNode::ReferenceRemovedEvent,this, SLOT(updateWidget()));
  updateWidget(); // if the scene is loaded before the widget is activated

  QTimer* timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(onUpdateTimer()));
  timer->start(1000);
}


void qSlicerROS2ModuleWidget::updateWidget()
{
  Q_D(qSlicerROS2ModuleWidget);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic || !logic->mDefaultROS2Node) {
    return;
  }

  refreshTopicsPanel();
  refreshParameterTable();
  refreshTf2Tables();

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

void qSlicerROS2ModuleWidget::onUpdateTimer()
{
  refreshTopicsPanel();
  refreshParameterTable();
  refreshTf2Tables();
}

// ── Observer helpers ────────────────────────────────────────────────────────
void qSlicerROS2ModuleWidget::observeSubscriber(vtkMRMLROS2SubscriberNode * node)
{
  if (mObservedSubscriber == node) return;
  if (mObservedSubscriber) {
    qvtkDisconnect(mObservedSubscriber, vtkCommand::ModifiedEvent, this, SLOT(updateTopicDetail()));
  }
  mObservedSubscriber = node;
  if (mObservedSubscriber) {
    qvtkConnect(mObservedSubscriber, vtkCommand::ModifiedEvent, this, SLOT(updateTopicDetail()));
    updateTopicDetail();
  }
}

void qSlicerROS2ModuleWidget::observeParameter(vtkMRMLROS2ParameterNode  * node)
{
  if (mObservedParameter == node) return;
  if (mObservedParameter) {
    qvtkDisconnect(mObservedParameter, vtkMRMLROS2ParameterNode::ParameterModifiedEvent, this, SLOT(updateParameterDetail()));
  }
  mObservedParameter = node;
  if (mObservedParameter) {
    qvtkConnect(mObservedParameter, vtkMRMLROS2ParameterNode::ParameterModifiedEvent, this, SLOT(updateParameterDetail()));
    updateParameterDetail();
  }
}

void qSlicerROS2ModuleWidget::observeTf2Lookup(vtkMRMLROS2Tf2LookupNode  * node)
{
  if (mObservedTf2Lookup == node) return;
  if (mObservedTf2Lookup) {
    qvtkDisconnect(mObservedTf2Lookup, vtkMRMLTransformNode::TransformModifiedEvent, this, SLOT(updateTf2Detail()));
  }
  mObservedTf2Lookup = node;
  if (mObservedTf2Lookup) {
    qvtkConnect(mObservedTf2Lookup, vtkMRMLTransformNode::TransformModifiedEvent, this, SLOT(updateTf2Detail()));
    updateTf2Detail();
  }
}

void qSlicerROS2ModuleWidget::onInnerTabChanged(int index)
{
  Q_D(qSlicerROS2ModuleWidget);
  QWidget* currentTab = d->tabWidget_2->widget(index);
  if (currentTab != d->tab_topics) {
    observeSubscriber(nullptr);
    d->rosSubscriberTableWidget->clearSelection();
    d->topicDetailGroupBox->setVisible(false);
  }
  if (currentTab != d->tab_parameters) {
    observeParameter(nullptr);
    d->parameterNodeTableWidget->clearSelection();
    d->parameterDetailGroupBox->setVisible(false);
  }
  if (currentTab != d->tab_tf2) {
    observeTf2Lookup(nullptr);
    d->tf2LookupTableWidget->clearSelection();
    d->tf2DetailGroupBox->setVisible(false);
  }
}

// ── Math helpers ────────────────────────────────────────────────────────────

void qSlicerROS2ModuleWidget::matrixToRPY(double m[4][4], double & roll, double & pitch, double & yaw)
{
  // ZYX Euler
  double sy = std::sqrt(m[0][0]*m[0][0] + m[1][0]*m[1][0]);
  bool singular = sy < 1e-6;
  if (!singular) {
    roll = std::atan2(m[2][1], m[2][2]);
    pitch = std::atan2(-m[2][0], sy);
    yaw = std::atan2(m[1][0], m[0][0]);
  } else {
    roll = std::atan2(-m[1][2], m[1][1]);
    pitch = std::atan2(-m[2][0], sy);
    yaw = 0;
  }
  // to degrees
  roll = roll * 180.0 / M_PI;
  pitch = pitch * 180.0 / M_PI;
  yaw = yaw * 180.0 / M_PI;
}

void qSlicerROS2ModuleWidget::matrixToQuaternion(double m[4][4], double & qw, double & qx, double & qy, double & qz)
{
  double tr = m[0][0] + m[1][1] + m[2][2];
  if (tr > 0) {
    double S = std::sqrt(tr+1.0) * 2; // S=4*qw 
    qw = 0.25 * S;
    qx = (m[2][1] - m[1][2]) / S;
    qy = (m[0][2] - m[2][0]) / S; 
    qz = (m[1][0] - m[0][1]) / S; 
  } else if ((m[0][0] > m[1][1])&&(m[0][0] > m[2][2])) {
    double S = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2; // S=4*qx 
    qw = (m[2][1] - m[1][2]) / S;
    qx = 0.25 * S;
    qy = (m[0][1] + m[1][0]) / S; 
    qz = (m[0][2] + m[2][0]) / S; 
  } else if (m[1][1] > m[2][2]) {
    double S = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2; // S=4*qy
    qw = (m[0][2] - m[2][0]) / S;
    qx = (m[0][1] + m[1][0]) / S; 
    qy = 0.25 * S;
    qz = (m[1][2] + m[2][1]) / S; 
  } else {
    double S = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2; // S=4*qz
    qw = (m[1][0] - m[0][1]) / S;
    qx = (m[0][2] + m[2][0]) / S;
    qy = (m[1][2] + m[2][1]) / S;
    qz = 0.25 * S;
  }
}

// ── Topics ──────────────────────────────────────────────────────────────────

void qSlicerROS2ModuleWidget::refreshTopicsPanel()
{
  Q_D(qSlicerROS2ModuleWidget);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic || !logic->mDefaultROS2Node) return;
  
  // Update subscribers
  size_t numSubs = logic->mDefaultROS2Node->GetNumberOfNodeReferences("subscriber");
  d->rosSubscriberTableWidget->setRowCount(numSubs);
  for (size_t index = 0; index < numSubs; ++index) {
    const char * id = logic->mDefaultROS2Node->GetNthNodeReferenceID("subscriber", index);
    vtkMRMLROS2SubscriberNode *sub = vtkMRMLROS2SubscriberNode::SafeDownCast(logic->mDefaultROS2Node->GetScene()->GetNodeByID(id));
    if (sub) {
      updateSubscriberTable(sub, index);
    }
  }

  // Update publishers
  size_t numPubs = logic->mDefaultROS2Node->GetNumberOfNodeReferences("publisher");
  d->rosPublisherTableWidget->setRowCount(numPubs);
  for (size_t index = 0; index < numPubs; ++index) {
    const char * id = logic->mDefaultROS2Node->GetNthNodeReferenceID("publisher", index);
    vtkMRMLROS2PublisherNode *pub = vtkMRMLROS2PublisherNode::SafeDownCast(logic->mDefaultROS2Node->GetScene()->GetNodeByID(id));
    if (pub) {
      updatePublisherTable(pub, index);
    }
  }
}

void qSlicerROS2ModuleWidget::updateSubscriberTable(vtkMRMLROS2SubscriberNode* sub, size_t row)
{
  Q_D(qSlicerROS2ModuleWidget);
  if (!d->rosSubscriberTableWidget->item(row, 0)) d->rosSubscriberTableWidget->setItem(row, 0, new QTableWidgetItem);
  if (!d->rosSubscriberTableWidget->item(row, 1)) d->rosSubscriberTableWidget->setItem(row, 1, new QTableWidgetItem);
  if (!d->rosSubscriberTableWidget->item(row, 2)) d->rosSubscriberTableWidget->setItem(row, 2, new QTableWidgetItem);
  
  d->rosSubscriberTableWidget->item(row, 0)->setText(QString::fromStdString(sub->GetTopic()));
  d->rosSubscriberTableWidget->item(row, 1)->setText(QString::fromStdString(sub->GetROSType()));
  d->rosSubscriberTableWidget->item(row, 2)->setText(QString::number(sub->GetNumberOfMessages()));
}

void qSlicerROS2ModuleWidget::updatePublisherTable(vtkMRMLROS2PublisherNode* pub, size_t row)
{
  Q_D(qSlicerROS2ModuleWidget);
  if (!d->rosPublisherTableWidget->item(row, 0)) d->rosPublisherTableWidget->setItem(row, 0, new QTableWidgetItem);
  if (!d->rosPublisherTableWidget->item(row, 1)) d->rosPublisherTableWidget->setItem(row, 1, new QTableWidgetItem);
  if (!d->rosPublisherTableWidget->item(row, 2)) d->rosPublisherTableWidget->setItem(row, 2, new QTableWidgetItem);
  
  d->rosPublisherTableWidget->item(row, 0)->setText(QString::fromStdString(pub->GetTopic()));
  d->rosPublisherTableWidget->item(row, 1)->setText(QString::fromStdString(pub->GetROSType()));
  d->rosPublisherTableWidget->item(row, 2)->setText(QString::number(pub->GetNumberOfMessagesSent()));
}

void qSlicerROS2ModuleWidget::subscriberRowSelected(int row, int col)
{
  Q_D(qSlicerROS2ModuleWidget);
  Q_UNUSED(col);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic || !logic->mDefaultROS2Node) return;
  
  QString topicName = d->rosSubscriberTableWidget->item(row, 0)->text();
  vtkMRMLROS2SubscriberNode *sub = logic->mDefaultROS2Node->GetSubscriberNodeByTopic(topicName.toStdString());
  
  d->rosPublisherTableWidget->clearSelection();
  observeSubscriber(sub);
}

void qSlicerROS2ModuleWidget::publisherRowSelected(int row, int col)
{
  Q_D(qSlicerROS2ModuleWidget);
  Q_UNUSED(col);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic || !logic->mDefaultROS2Node) return;
  
  d->rosSubscriberTableWidget->clearSelection();
  observeSubscriber(nullptr);
  
  QString topicName = d->rosPublisherTableWidget->item(row, 0)->text();
  vtkMRMLROS2PublisherNode *pub = logic->mDefaultROS2Node->GetPublisherNodeByTopic(topicName.toStdString());
  
  if (pub) {
    d->topicDetailGroupBox->setVisible(true);
    d->topicDetailHeaderLabel->setText(QStringLiteral("Topic: %1  [%2]").arg(topicName).arg(QString::fromUtf8(pub->GetROSType())));
    d->topicDetailTextEdit->setPlainText(QStringLiteral("Messages sent: %1").arg(pub->GetNumberOfMessagesSent()));
  }
}

void qSlicerROS2ModuleWidget::updateTopicDetail()
{
  Q_D(qSlicerROS2ModuleWidget);
  if (!mObservedSubscriber) return;
  
  d->topicDetailGroupBox->setVisible(true);
  QString topicName = QString::fromStdString(mObservedSubscriber->GetTopic());
  QString rosType = QString::fromUtf8(mObservedSubscriber->GetROSType());
  
  d->topicDetailHeaderLabel->setText(QStringLiteral("Topic: %1  [%2]").arg(topicName).arg(rosType));
  
  const bool skipYAML = rosType.contains("PointCloud") || rosType.contains("Image") || rosType.contains("LaserScan");
  
  if (!skipYAML) {
    constexpr size_t kMaxYAMLChars = 10000;
    std::string yaml = mObservedSubscriber->GetLastMessageYAML();
    if (yaml.size() > kMaxYAMLChars) {
      yaml = yaml.substr(0, kMaxYAMLChars) + "\n\n[truncated]";
    }
    d->topicDetailTextEdit->setPlainText(QString::fromStdString(yaml));
  } else {
    d->topicDetailTextEdit->setPlainText(QStringLiteral("(Message preview disabled for high-volume type)"));
  }
}

// ── Parameters ──────────────────────────────────────────────────────────────

void qSlicerROS2ModuleWidget::refreshParameterTable()
{
  Q_D(qSlicerROS2ModuleWidget);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic || !logic->mDefaultROS2Node) return;
  
  size_t numParams = logic->mDefaultROS2Node->GetNumberOfNodeReferences("parameter");
  d->parameterNodeTableWidget->setRowCount(numParams);
  
  for (size_t index = 0; index < numParams; ++index) {
    const char * id = logic->mDefaultROS2Node->GetNthNodeReferenceID("parameter", index);
    vtkMRMLROS2ParameterNode *param = vtkMRMLROS2ParameterNode::SafeDownCast(logic->mDefaultROS2Node->GetScene()->GetNodeByID(id));
    if (param) {
      if (!d->parameterNodeTableWidget->item(index, 0)) d->parameterNodeTableWidget->setItem(index, 0, new QTableWidgetItem);
      if (!d->parameterNodeTableWidget->item(index, 1)) d->parameterNodeTableWidget->setItem(index, 1, new QTableWidgetItem);
      if (!d->parameterNodeTableWidget->item(index, 2)) d->parameterNodeTableWidget->setItem(index, 2, new QTableWidgetItem);
      
      d->parameterNodeTableWidget->item(index, 0)->setText(QString::fromStdString(param->GetMonitoredNodeName()));
      d->parameterNodeTableWidget->item(index, 1)->setText(param->IsMonitoredNodeReady() ? "Ready" : "Waiting");
      d->parameterNodeTableWidget->item(index, 2)->setText(QString::number(param->GetMonitoredParameterNames().size()));
    }
  }
}

void qSlicerROS2ModuleWidget::parameterNodeRowSelected(int row, int col)
{
  Q_D(qSlicerROS2ModuleWidget);
  Q_UNUSED(col);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic || !logic->mDefaultROS2Node) return;
  
  QString nodeName = d->parameterNodeTableWidget->item(row, 0)->text();
  vtkMRMLROS2ParameterNode *param = logic->mDefaultROS2Node->GetParameterNodeByNode(nodeName.toStdString());
  
  observeParameter(param);
}

void qSlicerROS2ModuleWidget::updateParameterDetail()
{
  Q_D(qSlicerROS2ModuleWidget);
  if (!mObservedParameter) return;
  
  d->parameterDetailGroupBox->setVisible(true);
  QString nodeName = QString::fromStdString(mObservedParameter->GetMonitoredNodeName());
  QString status = mObservedParameter->IsMonitoredNodeReady() ? "Ready" : "Waiting";
  d->parameterDetailNodeLabel->setText(QStringLiteral("Node: %1 [%2]").arg(nodeName).arg(status));
  
  auto paramNames = mObservedParameter->GetMonitoredParameterNames();
  d->parameterDetailTableWidget->setRowCount(paramNames.size());
  
  for (size_t i = 0; i < paramNames.size(); ++i) {
    if (!d->parameterDetailTableWidget->item(i, 0)) d->parameterDetailTableWidget->setItem(i, 0, new QTableWidgetItem);
    if (!d->parameterDetailTableWidget->item(i, 1)) d->parameterDetailTableWidget->setItem(i, 1, new QTableWidgetItem);
    if (!d->parameterDetailTableWidget->item(i, 2)) d->parameterDetailTableWidget->setItem(i, 2, new QTableWidgetItem);
    
    std::string name = paramNames[i];
    d->parameterDetailTableWidget->item(i, 0)->setText(QString::fromStdString(name));
    d->parameterDetailTableWidget->item(i, 1)->setText(QString::fromStdString(mObservedParameter->GetParameterType(name)));
    d->parameterDetailTableWidget->item(i, 2)->setText(QString::fromStdString(mObservedParameter->PrintParameter(name)));
  }
}

// ── Tf2 ─────────────────────────────────────────────────────────────────────

void qSlicerROS2ModuleWidget::refreshTf2Tables()
{
  Q_D(qSlicerROS2ModuleWidget);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic || !logic->mDefaultROS2Node) return;
  
  size_t numLookups = logic->mDefaultROS2Node->GetNumberOfNodeReferences("lookup");
  d->tf2LookupTableWidget->setRowCount(numLookups);
  for (size_t index = 0; index < numLookups; ++index) {
    const char * id = logic->mDefaultROS2Node->GetNthNodeReferenceID("lookup", index);
    vtkMRMLROS2Tf2LookupNode *lookup = vtkMRMLROS2Tf2LookupNode::SafeDownCast(logic->mDefaultROS2Node->GetScene()->GetNodeByID(id));
    if (lookup) {
      if (!d->tf2LookupTableWidget->item(index, 0)) d->tf2LookupTableWidget->setItem(index, 0, new QTableWidgetItem);
      if (!d->tf2LookupTableWidget->item(index, 1)) d->tf2LookupTableWidget->setItem(index, 1, new QTableWidgetItem);
      if (!d->tf2LookupTableWidget->item(index, 2)) d->tf2LookupTableWidget->setItem(index, 2, new QTableWidgetItem);
      
      d->tf2LookupTableWidget->item(index, 0)->setText(QString::fromStdString(lookup->GetParentID()));
      d->tf2LookupTableWidget->item(index, 1)->setText(QString::fromStdString(lookup->GetChildID()));
      d->tf2LookupTableWidget->item(index, 2)->setText(QString::number(lookup->GetLookupAttempts()));
    }
  }
  
  size_t numBroadcasters = logic->mDefaultROS2Node->GetNumberOfNodeReferences("broadcaster");
  d->tf2BroadcasterTableWidget->setRowCount(numBroadcasters);
  for (size_t index = 0; index < numBroadcasters; ++index) {
    const char * id = logic->mDefaultROS2Node->GetNthNodeReferenceID("broadcaster", index);
    vtkMRMLROS2Tf2BroadcasterNode *bc = vtkMRMLROS2Tf2BroadcasterNode::SafeDownCast(logic->mDefaultROS2Node->GetScene()->GetNodeByID(id));
    if (bc) {
      if (!d->tf2BroadcasterTableWidget->item(index, 0)) d->tf2BroadcasterTableWidget->setItem(index, 0, new QTableWidgetItem);
      if (!d->tf2BroadcasterTableWidget->item(index, 1)) d->tf2BroadcasterTableWidget->setItem(index, 1, new QTableWidgetItem);
      if (!d->tf2BroadcasterTableWidget->item(index, 2)) d->tf2BroadcasterTableWidget->setItem(index, 2, new QTableWidgetItem);
      
      d->tf2BroadcasterTableWidget->item(index, 0)->setText(QString::fromStdString(bc->GetParentID()));
      d->tf2BroadcasterTableWidget->item(index, 1)->setText(QString::fromStdString(bc->GetChildID()));
      d->tf2BroadcasterTableWidget->item(index, 2)->setText(QString::number(bc->GetNumberOfBroadcasts()));
    }
  }
}

void qSlicerROS2ModuleWidget::tf2LookupRowSelected(int row, int col)
{
  Q_D(qSlicerROS2ModuleWidget);
  Q_UNUSED(col);
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic || !logic->mDefaultROS2Node) return;
  
  QString parentFrame = d->tf2LookupTableWidget->item(row, 0)->text();
  QString childFrame = d->tf2LookupTableWidget->item(row, 1)->text();
  vtkMRMLROS2Tf2LookupNode *lookup = logic->mDefaultROS2Node->GetTf2LookupNodeByParentChild(parentFrame.toStdString(), childFrame.toStdString());
  
  observeTf2Lookup(lookup);
}

void qSlicerROS2ModuleWidget::updateTf2Detail()
{
  Q_D(qSlicerROS2ModuleWidget);
  if (!mObservedTf2Lookup) return;
  
  d->tf2DetailGroupBox->setVisible(true);
  d->tf2DetailHeaderLabel->setText(QStringLiteral("Lookup: %1 → %2")
    .arg(QString::fromStdString(mObservedTf2Lookup->GetParentID()))
    .arg(QString::fromStdString(mObservedTf2Lookup->GetChildID())));
    
  vtkNew<vtkMatrix4x4> mat;
  if (!mObservedTf2Lookup->GetMatrixTransformToParent(mat.GetPointer())) {
    d->tf2DetailTextEdit->setPlainText("No transform data available.");
    return;
  }
  
  double m[4][4];
  for (int i=0; i<4; i++) for (int j=0; j<4; j++) m[i][j] = mat->GetElement(i, j);
  
  QString text;
  if (mTf2DisplayMode == Tf2DisplayMode::Matrix) {
    text = QStringLiteral("%1 %2 %3 %4\n%5 %6 %7 %8\n%9 %10 %11 %12\n%13 %14 %15 %16")
      .arg(m[0][0], 8, 'f', 4).arg(m[0][1], 8, 'f', 4).arg(m[0][2], 8, 'f', 4).arg(m[0][3], 8, 'f', 4)
      .arg(m[1][0], 8, 'f', 4).arg(m[1][1], 8, 'f', 4).arg(m[1][2], 8, 'f', 4).arg(m[1][3], 8, 'f', 4)
      .arg(m[2][0], 8, 'f', 4).arg(m[2][1], 8, 'f', 4).arg(m[2][2], 8, 'f', 4).arg(m[2][3], 8, 'f', 4)
      .arg(m[3][0], 8, 'f', 4).arg(m[3][1], 8, 'f', 4).arg(m[3][2], 8, 'f', 4).arg(m[3][3], 8, 'f', 4);
    d->tf2DisplayModeButton->setText("Matrix 4x4");
  } else if (mTf2DisplayMode == Tf2DisplayMode::RPY) {
    double roll, pitch, yaw;
    matrixToRPY(m, roll, pitch, yaw);
    text = QStringLiteral("Position (X,Y,Z):\n  %1  %2  %3\n\nRotation RPY (deg):\n  %4  %5  %6")
      .arg(m[0][3], 8, 'f', 4).arg(m[1][3], 8, 'f', 4).arg(m[2][3], 8, 'f', 4)
      .arg(roll, 8, 'f', 4).arg(pitch, 8, 'f', 4).arg(yaw, 8, 'f', 4);
    d->tf2DisplayModeButton->setText("RPY + XYZ");
  } else {
    double qw, qx, qy, qz;
    matrixToQuaternion(m, qw, qx, qy, qz);
    text = QStringLiteral("Position (X,Y,Z):\n  %1  %2  %3\n\nQuaternion (W,X,Y,Z):\n  %4  %5  %6  %7")
      .arg(m[0][3], 8, 'f', 4).arg(m[1][3], 8, 'f', 4).arg(m[2][3], 8, 'f', 4)
      .arg(qw, 8, 'f', 4).arg(qx, 8, 'f', 4).arg(qy, 8, 'f', 4).arg(qz, 8, 'f', 4);
    d->tf2DisplayModeButton->setText("Unit Quaternion");
  }
  
  d->tf2DetailTextEdit->setPlainText(text);
}

void qSlicerROS2ModuleWidget::setTf2DisplayMode(int mode)
{
  if (mode == 0) mTf2DisplayMode = Tf2DisplayMode::Matrix;
  else if (mode == 1) mTf2DisplayMode = Tf2DisplayMode::RPY;
  else if (mode == 2) mTf2DisplayMode = Tf2DisplayMode::Quaternion;
  
  updateTf2Detail();
}


// ── Robots (Existing) ───────────────────────────────────────────────────────

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
