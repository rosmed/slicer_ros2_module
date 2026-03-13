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
#include <QCheckBox> 
#include <QTimer>

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
#include <vtkMRMLModelNode.h>
#include <vtkMRMLModelDisplayNode.h>
#include <vtkMRMLLinearTransformNode.h>
#include <vtkMRMLROS2Tf2LookupNode.h>

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
                                     loadRobotButton, removeRobotButton,
                                     robotWidgetUi->ghostcheckBox);
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
                                                 QPushButton * removeRobotButton,
                                                 QCheckBox * ghostcheckBox)
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

  // If ghost is requested at load time, schedule creation after robot loads
  if (ghostcheckBox && ghostcheckBox->isChecked()) {
    std::cout << "============================================" << std::endl;
    std::cout << "GHOST CHECKBOX IS CHECKED!" << std::endl;
    std::cout << "Scheduling ghost creation for: " << robotNameLineEdit->text().toStdString() << std::endl;
    std::cout << "============================================" << std::endl;
    QTimer::singleShot(750, this, [=]() {
      onGhostToggled(robotNameLineEdit, true);
    });
  } else {
    std::cout << "Ghost checkbox is NOT checked or is null" << std::endl;
  }
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


void qSlicerROS2ModuleWidget::onGhostToggled(QLineEdit* robotNameLineEdit, bool enabled)
{
  vtkSlicerROS2Logic* logic = vtkSlicerROS2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }

  auto nodeNode = logic->GetDefaultROS2Node();
  if (!nodeNode) {
    qWarning() << Q_FUNC_INFO << " failed: Default ROS2 node missing";
    return;
  }

  const std::string robotName = robotNameLineEdit->text().toStdString();
  vtkMRMLROS2RobotNode* robot = nodeNode->GetRobotNodeByName(robotName);
  if (!robot) {
    qWarning() << Q_FUNC_INFO << " failed: Robot not found in scene: " << robotName.c_str();
    return;
  }

  vtkMRMLScene* scene = logic->GetMRMLScene();
  if (!scene) {
    qWarning() << Q_FUNC_INFO << " failed: MRML scene missing";
    return;
  }

  // Helper to remove all existing ghost models and transforms
  auto removeGhosts = [&]() {
    // Remove ghost models
    int ghostCount = robot->GetNumberOfNodeReferences("ghost_model");
    for (int i = ghostCount - 1; i >= 0; --i) {
      vtkMRMLModelNode* ghost = vtkMRMLModelNode::SafeDownCast(robot->GetNthNodeReference("ghost_model", i));
      if (ghost) {
        scene->RemoveNode(ghost);
      }
    }
    // Remove ghost transforms
    int transformCount = robot->GetNumberOfNodeReferences("ghost_transform");
    for (int i = transformCount - 1; i >= 0; --i) {
      vtkMRMLLinearTransformNode* transform = vtkMRMLLinearTransformNode::SafeDownCast(robot->GetNthNodeReference("ghost_transform", i));
      if (transform) {
        scene->RemoveNode(transform);
      }
    }
  };

  if (!enabled) {
    std::cout << "Removing ghost models and transforms for: " << robotName << std::endl;
    removeGhosts();
    return;
  }

  // Re-create ghosts from current models
  std::cout << "Creating ghost models for: " << robotName << std::endl;
  removeGhosts();

  int modelCount = robot->GetNumberOfNodeReferences("model");
  std::cout << "Found " << modelCount << " model nodes to duplicate" << std::endl;
  // Keep track of ghost transforms by link index to build hierarchy later
  std::vector<vtkSmartPointer<vtkMRMLLinearTransformNode>> ghostTransforms;
  ghostTransforms.reserve(modelCount);

  for (int i = 0; i < modelCount; ++i) {
    vtkMRMLModelNode* original = vtkMRMLModelNode::SafeDownCast(robot->GetNthNodeReference("model", i));
    if (!original) { continue; }

    // Create a separate transform node for this ghost link
    vtkSmartPointer<vtkMRMLLinearTransformNode> ghostTransform = vtkSmartPointer<vtkMRMLLinearTransformNode>::New();
    scene->AddNode(ghostTransform);
    std::string transformName = std::string(original->GetName() ? original->GetName() : "model") + "_ghost_transform";
    ghostTransform->SetName(transformName.c_str());
    
    // Initialize ghost transform to match original transform's current state
    vtkMRMLLinearTransformNode* origTransform = vtkMRMLLinearTransformNode::SafeDownCast(
      scene->GetNodeByID(original->GetTransformNodeID()));
    if (origTransform) {
      vtkNew<vtkMatrix4x4> matrix;
      origTransform->GetMatrixTransformToParent(matrix);
      ghostTransform->SetMatrixTransformToParent(matrix);
    }
    // Save transform for hierarchy wiring
    ghostTransforms.push_back(ghostTransform);

    // Create the ghost model
    vtkSmartPointer<vtkMRMLModelNode> ghost = vtkSmartPointer<vtkMRMLModelNode>::New();
    scene->AddNode(ghost);

    // Name: add _ghost suffix
    std::string ghostName = std::string(original->GetName() ? original->GetName() : "model") + "_ghost";
    ghost->SetName(ghostName.c_str());
    std::cout << "  Creating ghost: " << ghostName << " with transform: " << transformName << std::endl;

    // Share mesh data (identical geometry)
    if (original->GetMesh()) {
      ghost->SetAndObserveMesh(original->GetMesh());
    }

    // Display node copy with distinct visual appearance
    vtkMRMLModelDisplayNode* origDisp = vtkMRMLModelDisplayNode::SafeDownCast(original->GetDisplayNode());
    vtkNew<vtkMRMLModelDisplayNode> ghostDisp;
    scene->AddNode(ghostDisp.GetPointer());
    if (origDisp) {
      ghostDisp->Copy(origDisp);
    }
    // Set ghost to cyan color with high opacity for clear visibility
    ghostDisp->SetColor(0.0, 1.0, 1.0);  // Cyan
    ghostDisp->SetOpacity(0.30);  // High opacity for visibility
    ghost->SetAndObserveDisplayNodeID(ghostDisp->GetID());

    // Attach ghost to its own independent transform
    ghost->SetAndObserveTransformNodeID(ghostTransform->GetID());

    // Track as ghost on the robot for cleanup
    robot->AddNodeReferenceID("ghost_model", ghost->GetID());
    robot->AddNodeReferenceID("ghost_transform", ghostTransform->GetID());
  }
  
  // Replicate original transform hierarchy onto ghost transforms
  int lookupCount = robot->GetNumberOfNodeReferences("lookup");
  for (int i = 0; i < lookupCount; ++i) {
    vtkMRMLROS2Tf2LookupNode* lookup = vtkMRMLROS2Tf2LookupNode::SafeDownCast(robot->GetNthNodeReference("lookup", i));
    if (!lookup) { continue; }
    std::string parentFrame = lookup->GetParentID();
    // Find the ghost transform whose original child matches this parent
    for (int j = 0; j < lookupCount; ++j) {
      vtkMRMLROS2Tf2LookupNode* potentialParent = vtkMRMLROS2Tf2LookupNode::SafeDownCast(robot->GetNthNodeReference("lookup", j));
      if (!potentialParent) { continue; }
      std::string childFrame = potentialParent->GetChildID();
      if (childFrame == parentFrame) {
        if (i < static_cast<int>(ghostTransforms.size()) && j < static_cast<int>(ghostTransforms.size())) {
          ghostTransforms[i]->SetAndObserveTransformNodeID(ghostTransforms[j]->GetID());
        }
        break;
      }
    }
  }

  // Sync ghost transforms to current TF2 lookup poses to avoid stacking at origin
  for (int i = 0; i < lookupCount && i < static_cast<int>(ghostTransforms.size()); ++i) {
    vtkMRMLROS2Tf2LookupNode* lookup = vtkMRMLROS2Tf2LookupNode::SafeDownCast(robot->GetNthNodeReference("lookup", i));
    if (!lookup) { continue; }
    vtkNew<vtkMatrix4x4> m;
    lookup->GetMatrixTransformToParent(m);
    ghostTransforms[i]->SetMatrixTransformToParent(m);
    ghostTransforms[i]->Modified();
  }

  // Schedule a short delayed re-sync to let TF2 warm up
  QTimer::singleShot(400, this, [=]() {
    int lc = robot->GetNumberOfNodeReferences("lookup");
    for (int i = 0; i < lc && i < static_cast<int>(ghostTransforms.size()); ++i) {
      vtkMRMLROS2Tf2LookupNode* lu = vtkMRMLROS2Tf2LookupNode::SafeDownCast(robot->GetNthNodeReference("lookup", i));
      if (!lu) { continue; }
      vtkNew<vtkMatrix4x4> mm;
      lu->GetMatrixTransformToParent(mm);
      ghostTransforms[i]->SetMatrixTransformToParent(mm);
      ghostTransforms[i]->Modified();
    }
  });
  
  std::cout << "Ghost creation complete!" << std::endl;
}
