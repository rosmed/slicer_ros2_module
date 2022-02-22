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

#ifndef __qSlicerSlicerRos2ModuleWidget_h
#define __qSlicerSlicerRos2ModuleWidget_h

// Slicer includes
#include "qSlicerAbstractModuleWidget.h"

#include "qSlicerSlicerRos2ModuleExport.h"

class qSlicerSlicerRos2ModuleWidgetPrivate;
class vtkMRMLNode;

/// \ingroup Slicer_QtModules_ExtensionTemplate
class Q_SLICER_QTMODULES_SLICERROS2_EXPORT qSlicerSlicerRos2ModuleWidget :
  public qSlicerAbstractModuleWidget
{
  Q_OBJECT

public:

  typedef qSlicerAbstractModuleWidget Superclass;
  qSlicerSlicerRos2ModuleWidget(QWidget *parent=0);
  virtual ~qSlicerSlicerRos2ModuleWidget();

public slots:

  //void setMRMLScene(vtkMRMLScene* scene);

protected:
  QScopedPointer<qSlicerSlicerRos2ModuleWidgetPrivate> d_ptr;

  void setup() override;

  /// Create and return the widget representation associated to this module
  //virtual qSlicerAbstractModuleRepresentation * createWidgetRepresentation();

  /// Create and return the logic associated to this module
  //virtual vtkMRMLAbstractLogic* createLogic();


protected slots:
  void onFileSelected(const QString&);


private:
  Q_DECLARE_PRIVATE(qSlicerSlicerRos2ModuleWidget);
  Q_DISABLE_COPY(qSlicerSlicerRos2ModuleWidget);
};

#endif
