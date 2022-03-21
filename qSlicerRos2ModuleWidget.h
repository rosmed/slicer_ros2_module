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

protected:
  QScopedPointer<qSlicerRos2ModuleWidgetPrivate> d_ptr;

  void setup() override;
  QTimer* mTimer;

  /// Create and return the widget representation associated to this module
  //virtual qSlicerAbstractModuleRepresentation * createWidgetRepresentation();

  /// Create and return the logic associated to this module
  //virtual vtkMRMLAbstractLogic* createLogic();


protected slots:
  void onFileSelected(const QString&);
  void onTimerTimeOut();
  void onClearSceneSelected();


private:
  Q_DECLARE_PRIVATE(qSlicerRos2ModuleWidget);
  Q_DISABLE_COPY(qSlicerRos2ModuleWidget);
};

#endif
