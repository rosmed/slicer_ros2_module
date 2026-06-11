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
class vtkMRMLROS2ParameterNode;
class vtkMRMLROS2Tf2LookupNode;
class QLineEdit;
class QPushButton;
class QCheckBox;
class QMenu;
class QAction;

/// \ingroup Slicer_QtModules_ExtensionTemplate
class Q_SLICER_QTMODULES_ROS2_EXPORT qSlicerROS2ModuleWidget :
  public qSlicerAbstractModuleWidget
{
  Q_OBJECT

  /// Current display mode for the TF2 detail panel.
  enum class Tf2DisplayMode { Matrix, RPY, Quaternion };

public:
  typedef qSlicerAbstractModuleWidget Superclass;
  qSlicerROS2ModuleWidget(QWidget *parent=0);
  virtual ~qSlicerROS2ModuleWidget();

protected:
  QScopedPointer<qSlicerROS2ModuleWidgetPrivate> d_ptr;

  void setup() override;

  // ── Observer helpers ──────────────────────────────────────────────────────
  void observeSubscriber(vtkMRMLROS2SubscriberNode * node);
  void observeParameter(vtkMRMLROS2ParameterNode  * node);
  void observeTf2Lookup(vtkMRMLROS2Tf2LookupNode  * node);

  // ── Transform decomposition helpers ───────────────────────────────────────
  static void matrixToRPY(double m[4][4],
                           double & roll, double & pitch, double & yaw);
  static void matrixToQuaternion(double m[4][4],
                                  double & qw, double & qx,
                                  double & qy, double & qz);

protected slots:
  // ── Existing / Robots ─────────────────────────────────────────────────────
  void updateWidget(void);
  void onUpdateTimer(void);
  void onAddNewRobotClicked(const std::string & robotName = "robot", bool active = false);
  void onLoadRobotClicked(QLineEdit * robotNameLineEdit,
                          QLineEdit * parameterNodeNameLineEdit,
                          QLineEdit * parameterNameLineEdit,
                          QLineEdit * fixedFrameLineEdit,
                          QLineEdit * tfPrefixLineEdit,
                          QPushButton * loadRobotButton,
                          QPushButton * removeRobotButton);
  void onRemoveRobotClicked(QLineEdit * robotNameLineEdit,
                            QLineEdit * parameterNodeNameLineEdit,
                            QLineEdit * parameterNameLineEdit,
                            QLineEdit * fixedFrameLineEdit,
                            QLineEdit * tfPrefixLineEdit,
                            QPushButton * loadRobotButton,
                            QPushButton * removeRobotButton,
                            QWidget * robotWidget);

  // ── Topics tab ────────────────────────────────────────────────────────────
  void refreshTopicsPanel(void);
  void updateSubscriberTable(vtkMRMLROS2SubscriberNode * sub, size_t row);
  void updatePublisherTable(vtkMRMLROS2PublisherNode  * pub, size_t row);
  void subscriberRowSelected(int row, int col);
  void publisherRowSelected(int row, int col);
  void updateTopicDetail(void);       // driven by MRML observer

  // ── Parameters tab ────────────────────────────────────────────────────────
  void refreshParameterTable(void);
  void parameterNodeRowSelected(int row, int col);
  void updateParameterDetail(void);   // driven by MRML observer

  // ── TF2 tab ───────────────────────────────────────────────────────────────
  void refreshTf2Tables(void);
  void tf2LookupRowSelected(int row, int col);
  void updateTf2Detail(void);         // driven by MRML observer
  void setTf2DisplayMode(int mode);   // called by QAction

  // ── Tab visibility gating ─────────────────────────────────────────────────
  void onInnerTabChanged(int index);

private:
  std::vector<std::string> robotsAddedToTheWidget;

  // Currently observed MRML nodes (one per detail panel)
  vtkMRMLROS2SubscriberNode * mObservedSubscriber = nullptr;
  vtkMRMLROS2ParameterNode  * mObservedParameter  = nullptr;
  vtkMRMLROS2Tf2LookupNode  * mObservedTf2Lookup  = nullptr;

  // TF2 detail display mode
  Tf2DisplayMode mTf2DisplayMode = Tf2DisplayMode::Matrix;

  Q_DECLARE_PRIVATE(qSlicerROS2ModuleWidget);
  Q_DISABLE_COPY(qSlicerROS2ModuleWidget);
};

#endif
