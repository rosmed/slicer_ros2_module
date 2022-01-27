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

// FooBar Widgets includes
#include "qSlicerSlicerRos2FooBarWidget.h"
#include "ui_qSlicerSlicerRos2FooBarWidget.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_SlicerRos2
class qSlicerSlicerRos2FooBarWidgetPrivate
  : public Ui_qSlicerSlicerRos2FooBarWidget
{
  Q_DECLARE_PUBLIC(qSlicerSlicerRos2FooBarWidget);
protected:
  qSlicerSlicerRos2FooBarWidget* const q_ptr;

public:
  qSlicerSlicerRos2FooBarWidgetPrivate(
    qSlicerSlicerRos2FooBarWidget& object);
  virtual void setupUi(qSlicerSlicerRos2FooBarWidget*);
};

// --------------------------------------------------------------------------
qSlicerSlicerRos2FooBarWidgetPrivate
::qSlicerSlicerRos2FooBarWidgetPrivate(
  qSlicerSlicerRos2FooBarWidget& object)
  : q_ptr(&object)
{
}

// --------------------------------------------------------------------------
void qSlicerSlicerRos2FooBarWidgetPrivate
::setupUi(qSlicerSlicerRos2FooBarWidget* widget)
{
  this->Ui_qSlicerSlicerRos2FooBarWidget::setupUi(widget);
}

//-----------------------------------------------------------------------------
// qSlicerSlicerRos2FooBarWidget methods

//-----------------------------------------------------------------------------
qSlicerSlicerRos2FooBarWidget
::qSlicerSlicerRos2FooBarWidget(QWidget* parentWidget)
  : Superclass( parentWidget )
  , d_ptr( new qSlicerSlicerRos2FooBarWidgetPrivate(*this) )
{
  Q_D(qSlicerSlicerRos2FooBarWidget);
  d->setupUi(this);
}

//-----------------------------------------------------------------------------
qSlicerSlicerRos2FooBarWidget
::~qSlicerSlicerRos2FooBarWidget()
{
}
