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

#ifndef __qSlicerROS2Module_h
#define __qSlicerROS2Module_h

// Slicer includes
#include "qSlicerLoadableModule.h"

#include "qSlicerROS2ModuleExport.h"

/// \ingroup Slicer_QtModules_ExtensionTemplate
class Q_SLICER_QTMODULES_ROS2_EXPORT
qSlicerROS2Module
  : public qSlicerLoadableModule
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "org.slicer.modules.loadable.qSlicerLoadableModule/1.0");
  Q_INTERFACES(qSlicerLoadableModule);

public:
  typedef qSlicerLoadableModule Superclass;
  explicit qSlicerROS2Module(QObject *parent=nullptr);
  ~qSlicerROS2Module() override;

  qSlicerGetTitleMacro(QTMODULE_TITLE);

  QString helpText() const override;
  QString acknowledgementText() const override;
  QStringList contributors() const override;

  QIcon icon() const override;

  QStringList categories() const override;
  QStringList dependencies() const override;

public slots:
  void stopTimer(void);

protected slots:
  void onTimerTimeOut(void);

protected:
  /// Initialize the module. Register the volumes reader/writer
  void setup() override;

  /// Create and return the widget representation associated to this module
  qSlicerAbstractModuleRepresentation * createWidgetRepresentation() override;

  /// Create and return the logic associated to this module
  vtkMRMLAbstractLogic* createLogic() override;

private:
  Q_DISABLE_COPY(qSlicerROS2Module);

  QTimer* mTimer;
};

#endif
