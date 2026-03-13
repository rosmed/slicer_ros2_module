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

#ifndef __qSlicerROS2ModuleWidget_h
#define __qSlicerROS2ModuleWidget_h

// Slicer includes
#include "qSlicerAbstractModuleWidget.h"
#include "qSlicerROS2ModuleExport.h"

// Qt includes
#include <QFileDialog>

class qSlicerROS2ModuleWidgetPrivate;
class vtkMRMLNode;
class vtkMRMLROS2SubscriberNode;
class vtkMRMLROS2PublisherNode;
class vtkMRMLROS2RobotNode;
class QLineEdit;
class QPushButton;
class QCheckBox;

/// \ingroup Slicer_QtModules_ExtensionTemplate
class Q_SLICER_QTMODULES_ROS2_EXPORT qSlicerROS2ModuleWidget :
  public qSlicerAbstractModuleWidget
{
  Q_OBJECT

public:
  typedef qSlicerAbstractModuleWidget Superclass;
  qSlicerROS2ModuleWidget(QWidget *parent=0);
  virtual ~qSlicerROS2ModuleWidget();

protected:
  QScopedPointer<qSlicerROS2ModuleWidgetPrivate> d_ptr;

  void setup() override;

protected slots:
  void updateWidget(void);
  void updateSubscriberTable(vtkMRMLROS2SubscriberNode * sub, size_t row);
  void updatePublisherTable(vtkMRMLROS2PublisherNode * sub, size_t row);
  void onAddNewRobotClicked(const std::string & robotName = "robot", bool active = false);
  void refreshSubTable(void);
  void refreshPubTable(void);

  // Slots for dynamic widgets
  void subscriberClicked(int row, int col);
  void publisherClicked(int row, int col);
  void onLoadRobotClicked(QLineEdit * robotNameLineEdit,
                          QLineEdit * parameterNodeNameLineEdit,
                          QLineEdit * parameterNameLineEdit,
                          QLineEdit * fixedFrameLineEdit,
                          QLineEdit * tfPrefixLineEdit,
                          QPushButton * loadRobotButton,
                          QPushButton * removeRobotButton,
                          QCheckBox * ghostcheckBox);
  void onRemoveRobotClicked(QLineEdit * robotNameLineEdit,
                            QLineEdit * parameterNodeNameLineEdit,
                            QLineEdit * parameterNameLineEdit,
                            QLineEdit * fixedFrameLineEdit,
                            QLineEdit * tfPrefixLineEdit,
                            QPushButton * loadRobotButton,
                            QPushButton * removeRobotButton,
                            QWidget * robotWidget);
  void onGhostToggled(QLineEdit * robotNameLineEdit, bool enabled);

private:
  std::vector<std::string> robotsAddedToTheWidget;
  Q_DECLARE_PRIVATE(qSlicerROS2ModuleWidget);
  Q_DISABLE_COPY(qSlicerROS2ModuleWidget);
};

#endif
