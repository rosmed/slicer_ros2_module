/*==============================================================================

  Program: 3D Slicer

  Copyright (c) Kitware Inc.

  See COPYRIGHT.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

  This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc.
  and was partially funded by NIH grant 3P41RR013218-12S1

==============================================================================*/

#ifndef __qSlicerSlicerRos2FooBarWidget_h
#define __qSlicerSlicerRos2FooBarWidget_h

// Qt includes
#include <QWidget>

// FooBar Widgets includes
#include "qSlicerSlicerRos2ModuleWidgetsExport.h"

class qSlicerSlicerRos2FooBarWidgetPrivate;

/// \ingroup Slicer_QtModules_SlicerRos2
class Q_SLICER_MODULE_SLICERROS2_WIDGETS_EXPORT qSlicerSlicerRos2FooBarWidget
  : public QWidget
{
  Q_OBJECT
public:
  typedef QWidget Superclass;
  qSlicerSlicerRos2FooBarWidget(QWidget *parent=0);
  ~qSlicerSlicerRos2FooBarWidget() override;

protected slots:

protected:
  QScopedPointer<qSlicerSlicerRos2FooBarWidgetPrivate> d_ptr;

private:
  Q_DECLARE_PRIVATE(qSlicerSlicerRos2FooBarWidget);
  Q_DISABLE_COPY(qSlicerSlicerRos2FooBarWidget);
};

#endif
