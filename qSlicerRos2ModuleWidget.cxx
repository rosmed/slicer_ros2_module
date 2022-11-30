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

  // Set up timer connections
  connect(mTimer, SIGNAL(timeout()), this, SLOT(onTimerTimeOut()));
  connect(qSlicerApplication::application(), SIGNAL(lastWindowClosed()), this, SLOT(stopTimer()));

  // Setup state / selection options
  // this->connect(d->stateSelectionComboBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(onStateSelection(const QString&)));

  // this->connect(d->descriptionSelectionComboBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(onDescriptionSelection(const QString&)));

  // Set up signals / slots for dynamically loaded widgets
  // Note: All of the QLineEdits are triggered by pressing enter in the edit box - the slot functions access the text that was entered themselves
  this->connect(d->loadVisualizationButton, SIGNAL(clicked(bool)), this, SLOT(onNodeOrParameterNameEntered()));
  this->connect(d->rosSubscriberTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(printLastMessage(int, int)));
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



void qSlicerRos2ModuleWidget::printLastMessage(int row, int col)
{
  // Row is a reference to the message index
  Q_D(qSlicerRos2ModuleWidget);
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  if (col == 2){ // only invoked when users click the number of messages cell
    QString subName = d->rosSubscriberTableWidget->item(row,0)->text();
    const char* referenceRole = subName.toStdString().c_str();
    vtkMRMLROS2SubscriberNode *sub = vtkMRMLROS2SubscriberNode::SafeDownCast(logic->mROS2Node->GetNodeReference(referenceRole));
    QString message = sub->GetLastMessageYAML().c_str();
    QLabel *popupLabel = new QLabel();
    popupLabel->setText(message);
    popupLabel->show();
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
  if (logic->mROS2Node && modifiedConnect == 0){
    qvtkReconnect(logic->mROS2Node, vtkCommand::ModifiedEvent, this, SLOT(updateWidget())); // Set up observer
    // qvtkReconnect(logic->mROS2Node, vtkMRMLNode::ReferencedNodeModifiedEvent, this, SLOT(updateSubscriberTableWidget())); // this should work?
    logic->mROS2Node->Modified(); // Invoke setup by calling modified once
    modifiedConnect++; // prevent second observer connection
  }
}


void qSlicerRos2ModuleWidget::updateWidget()
{
  std::cerr << "Table widget updated" << std::endl;
  Q_D(qSlicerRos2ModuleWidget);
  this->Superclass::setup();
  vtkSlicerRos2Logic* logic = vtkSlicerRos2Logic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << " failed: Invalid SlicerROS2 logic";
    return;
  }
  // Shouldn't need list should just get from the reference
  size_t subRow = 0;
  size_t pubRow = 0;
  std::cerr << "Num refs: " << logic->mROS2Node->GetNumberOfNodeReferenceRoles() << std::endl;
  d->rosPublisherTableWidget->clearContents();
  d->rosSubscriberTableWidget->clearContents();

  for (int j = 0; j < (logic->mROS2Node->GetNumberOfNodeReferenceRoles()); j++){

    std::string subName = logic->mROS2Node->GetNthNodeReferenceRole(j);
    std::string pubString = "pub";
    std::string subString = "sub";
    
    if (subName.find(subString) != std::string::npos) {
      std::cout << "Subsriber!" << '\n';
      vtkMRMLROS2SubscriberNode *sub = vtkMRMLROS2SubscriberNode::SafeDownCast(logic->mROS2Node->GetNodeReference(subName.c_str()));
      updateSubscriberTable(sub, subRow);
      std::cerr << "Num sub rows: " << subRow << std::endl;
      subRow++;
    }
  
    else if (subName.find(pubString) != std::string::npos){
      std::cout << "Publisher!" << '\n';
      vtkMRMLROS2PublisherNode *sub = vtkMRMLROS2PublisherNode::SafeDownCast(logic->mROS2Node->GetNodeReference(subName.c_str()));
      updatePublisherTable(sub, pubRow);
      std::cerr << "Num pub rows: " << pubRow << std::endl;
      pubRow++;
    }
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
  QString numMessages = QVariant(static_cast<int>(sub->GetNumberOfMessages())).toString(); // to convert to an int and then string

  QTableWidgetItem *topic_item = d->rosSubscriberTableWidget->item(row, 0);
  QTableWidgetItem *num_messages_item = d->rosSubscriberTableWidget->item(row, 1);
  QTableWidgetItem *type_item = d->rosSubscriberTableWidget->item(row, 2);

  std::cerr << topicName.toStdString() << std::endl;
  // if the row doesn't exist, populate
  if (!topic_item) {
    topic_item = new QTableWidgetItem;
    d->rosSubscriberTableWidget->setItem(row, 0, topic_item);
    topic_item->setText(topicName);
    num_messages_item = new QTableWidgetItem;
    d->rosSubscriberTableWidget->setItem(row, 1, num_messages_item);
    num_messages_item->setText(numMessages);
    type_item = new QTableWidgetItem;
    d->rosSubscriberTableWidget->setItem(row, 2, type_item);
    type_item->setText(typeName);
    }
    else {
      num_messages_item->setText(numMessages);
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
  QString numMessages = QVariant(static_cast<int>(sub->GetNumberOfMessages())).toString(); // to convert to an int and then string

  QTableWidgetItem *topic_item = d->rosPublisherTableWidget->item(row, 0);
  QTableWidgetItem *num_messages_item = d->rosPublisherTableWidget->item(row, 1);
  QTableWidgetItem *type_item = d->rosPublisherTableWidget->item(row, 2);

  // if the row doesn't exist, populate
  if (!topic_item) {
    topic_item = new QTableWidgetItem;
    d->rosPublisherTableWidget->setItem(row, 0, topic_item);
    topic_item->setText(topicName);
    num_messages_item = new QTableWidgetItem;
    d->rosPublisherTableWidget->setItem(row, 1, num_messages_item);
    type_item = new QTableWidgetItem;
    d->rosPublisherTableWidget->setItem(row, 2, type_item);
    type_item->setText(typeName);
    }
    else {
      num_messages_item->setText(numMessages);
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
  // d->rosSubscriberTableWidget->setRowCount(logic->numSubs);
  d->rosSubscriberTableWidget->setRowCount(5);

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
  // d->rosPublisherTableWidget->setRowCount(logic->numPubs);
  d->rosPublisherTableWidget->setRowCount(5);
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
    // d->stateWidgetGroupBox->hide();
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
