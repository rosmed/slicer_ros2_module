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
#include <QButtonGroup>

// Slicer includes
#include "qSlicerSlicerRos2ModuleWidget.h"
#include "ui_qSlicerSlicerRos2ModuleWidget.h"

// reference to Logic
#include "vtkSlicerSlicerRos2Logic.h"

// Slicer includes
#include "vtkMRMLModelDisplayNode.h"
#include "vtkMRMLMarkupsDisplayNode.h"
#include "vtkMRMLDisplayNode.h"
#include "vtkMRMLModelNode.h"
#include "vtkMRMLMarkupsFiducialNode.h"
#include "vtkMRMLInteractionNode.h"
#include "vtkMRMLScene.h"

// KDL include_directories
#include "kdl_parser/kdl_parser.hpp"
#include<iostream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
using namespace std;
using namespace KDL;

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerSlicerRos2ModuleWidgetPrivate: public Ui_qSlicerSlicerRos2ModuleWidget
{

public:
  qSlicerSlicerRos2ModuleWidgetPrivate();
  vtkSlicerSlicerRos2Logic* logic() const;
};


//-----------------------------------------------------------------------------
qSlicerSlicerRos2ModuleWidgetPrivate::qSlicerSlicerRos2ModuleWidgetPrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerSlicerRos2ModuleWidget methods

//-----------------------------------------------------------------------------
qSlicerSlicerRos2ModuleWidget::qSlicerSlicerRos2ModuleWidget(QWidget* _parent)
  : Superclass( _parent )
  , d_ptr( new qSlicerSlicerRos2ModuleWidgetPrivate )
{
}

//-----------------------------------------------------------------------------
qSlicerSlicerRos2ModuleWidget::~qSlicerSlicerRos2ModuleWidget()
{
}

//-----------------------------------------------------------------------------
void qSlicerSlicerRos2ModuleWidget::setup()
{
  Q_D(qSlicerSlicerRos2ModuleWidget);
  d->setupUi(this);
  this->Superclass::setup();

  // Setup connection for the button
  this->connect(d->PrintTreeButton, SIGNAL(clicked(bool)), this, SLOT(onPrintTreeButton()));
}

void qSlicerSlicerRos2ModuleWidget::onPrintTreeButton()
{
  Q_D(qSlicerSlicerRos2ModuleWidget);
  this->Superclass::setup();
  // Load the urdf file into a KDL tree
  KDL::Tree my_tree;
  if (!kdl_parser::treeFromFile("/home/laura/ros2_ws/src/slicer_ros/models/omni.urdf", my_tree)){
    qCritical() << Q_FUNC_INFO << "No urdf file to load.";
  }

  // Check if the mrml scene exists/ is valid
  if (this->mrmlScene() == NULL){
    qCritical() << "Invalid scene!";
    return;
  }

}
