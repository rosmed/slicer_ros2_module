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

#ifndef __qSlicerRos2ModuleWidget_h
#define __qSlicerRos2ModuleWidget_h

// Slicer includes
#include "qSlicerAbstractModuleWidget.h"
#include "qSlicerRos2ModuleExport.h"

// Qt includes
#include <QLineEdit>
#include <QFileDialog>
#include <QPushButton>

class qSlicerRos2ModuleWidgetPrivate;
class vtkMRMLNode;

/// \ingroup Slicer_QtModules_ExtensionTemplate
class Q_SLICER_QTMODULES_ROS2_EXPORT qSlicerRos2ModuleWidget :
  public qSlicerAbstractModuleWidget
{
  Q_OBJECT

public:

  typedef qSlicerAbstractModuleWidget Superclass;
  qSlicerRos2ModuleWidget(QWidget *parent=0);
  virtual ~qSlicerRos2ModuleWidget();

public slots:

  //void setMRMLScene(vtkMRMLScene* scene);
  void stopSound();

protected:
  QScopedPointer<qSlicerRos2ModuleWidgetPrivate> d_ptr;

  void setup() override;
  QTimer* mTimer;
  bool timerOff = false;

  QLineEdit *topicLineEdit = new QLineEdit(tr("/joint_states"));
  QFileDialog *urdfFileSelector = new QFileDialog(); // Was a QComboBox we populated - is the File dialog too complicated? - should we do this: https://doc.qt.io/qt-5/qtwidgets-dialogs-findfiles-example.html
  QLineEdit *nodeLineEdit = new QLineEdit(tr("/robot_state_publisher"));
  QLineEdit *paramLineEdit = new QLineEdit(tr("robot_description"));
  QPushButton *loadModelButton = new QPushButton();
  QPushButton *selectFileButton = new QPushButton();

  /// Create and return the widget representation associated to this module
  //virtual qSlicerAbstractModuleRepresentation * createWidgetRepresentation();

  /// Create and return the logic associated to this module
  //virtual vtkMRMLAbstractLogic* createLogic();

protected slots:
  void onFileSelected(const QString&);
  void onStateSelection(const QString&);
  void onDescriptionSelection(const QString&);
  void onTimerTimeOut(void);
  void onClearSceneSelected(void);

  // Slots for dyanmic widgets
  void onTopicNameEntered(void);
  void onNodeOrParameterNameEntered(void);
  void onDescriptionFileSelected(void);
  void onLoadModelButtonSelected(void);
  void onSelectFile(void);
  void onBroadcastButtonPressed();

private:
  Q_DECLARE_PRIVATE(qSlicerRos2ModuleWidget);
  Q_DISABLE_COPY(qSlicerRos2ModuleWidget);
};

#endif
