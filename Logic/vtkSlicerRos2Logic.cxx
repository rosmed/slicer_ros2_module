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

// ROS includes
#include <rclcpp/rclcpp.hpp>

// SlicerRos2 Logic includes
#include "vtkSlicerRos2Logic.h"

// MRML includes
#include <vtkMRMLScene.h>
#include <vtkMRMLModelNode.h>
#include <vtkMRMLTransformStorageNode.h>
#include <vtkMRMLTransformNode.h>
#include <vtkMRMLLinearTransformNode.h>

// VTK includes
#include <vtkMatrix4x4.h>
#include <vtkMatrix3x3.h>
#include <vtkMath.h>
#include <vtkTransform.h>

#include <qSlicerCoreIOManager.h>

// KDL includes
#include "kdl_parser/kdl_parser.hpp"
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <urdf/model.h>

// Python includes
#include "vtkSlicerConfigure.h"
#ifdef Slicer_USE_PYTHONQT
#include "PythonQt.h"
#endif

// Generic includes
#include <boost/filesystem/path.hpp>



//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlicerRos2Logic);

//----------------------------------------------------------------------------
vtkSlicerRos2Logic::vtkSlicerRos2Logic()
{
  typedef char * char_pointer;
  char_pointer * argv = new char_pointer[1];
  const std::string nodeName = GetClassName();
  argv[0]= new char[nodeName.size() + 1];
  strcpy(argv[0], nodeName.c_str());
  int argc = 1;
  rclcpp::init(argc, argv);

  // parameter
  mNodePointer = std::make_shared<rclcpp::Node>(nodeName);
  mParameterClient
    = std::make_shared<rclcpp::AsyncParametersClient>
    (mNodePointer,
     "/robot_state_publisher");

  std::chrono::seconds sec(1);
  mParameterClient->wait_for_service(sec);
  auto parameters_future
    = mParameterClient->get_parameters
    ({"robot_description"},
     std::bind(&vtkSlicerRos2Logic::ParameterCallback,
               this, std::placeholders::_1));
  // subscription
  mJointStateSubscription
    = mNodePointer->create_subscription<sensor_msgs::msg::JointState>
    ("/joint_states", 10, std::bind(&vtkSlicerRos2Logic::JointStateCallback,
				  this, std::placeholders::_1));

  mTfBuffer = std::make_unique<tf2_ros::Buffer>(mNodePointer->get_clock());
  mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

}


//----------------------------------------------------------------------------
vtkSlicerRos2Logic::~vtkSlicerRos2Logic()
{
  rclcpp::shutdown();
}

//----------------------------------------------------------------------------
void vtkSlicerRos2Logic::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//---------------------------------------------------------------------------
void vtkSlicerRos2Logic::SetMRMLSceneInternal(vtkMRMLScene * newScene)
{
  vtkNew<vtkIntArray> events;
  events->InsertNextValue(vtkMRMLScene::NodeAddedEvent);
  events->InsertNextValue(vtkMRMLScene::NodeRemovedEvent);
  events->InsertNextValue(vtkMRMLScene::EndBatchProcessEvent);
  this->SetAndObserveMRMLSceneEventsInternal(newScene, events.GetPointer());
}

//-----------------------------------------------------------------------------
void vtkSlicerRos2Logic::RegisterNodes()
{
  assert(this->GetMRMLScene() != 0);
}

//---------------------------------------------------------------------------
void vtkSlicerRos2Logic::UpdateFromMRMLScene()
{
  assert(this->GetMRMLScene() != 0);
}

//---------------------------------------------------------------------------
void vtkSlicerRos2Logic
::OnMRMLSceneNodeAdded(vtkMRMLNode* vtkNotUsed(node))
{
}

//---------------------------------------------------------------------------
void vtkSlicerRos2Logic
::OnMRMLSceneNodeRemoved(vtkMRMLNode* vtkNotUsed(node))
{
}

//----------------------------------------------------------------------------
void vtkSlicerRos2Logic
::loadRobotSTLModels()
{

  if (parameterNodeCallbackFlag == false){
    std::cout << " Never entered call back" << std::endl;
    return;
  }
  // Print out the urdf from param
  std::cout << robot_description_string << std::endl;
  // Parser the urdf file into an urdf model - to get names of links and pos/ rpy
  urdf::Model my_model;
  if (!my_model.initString(robot_description_string)) {
      return;
  }

  // load urdf file into a kdl tree to do forward kinematics
  KDL::Tree my_tree;
  if (!kdl_parser::treeFromString(robot_description_string, my_tree)) {
    return;
  }

  // Start by getting the name of the root link and add it to the vector of strings
  std::shared_ptr<const urdf::Link> root = my_model.getRoot();
  std::string root_name = root->name;
  std::vector<std::string> link_names_vector;
  link_names_vector.push_back(root_name);
  std::vector< std::shared_ptr< urdf::Visual > > visual_vector;
  visual_vector.push_back(root->visual);


  while (true) {
    std::shared_ptr<const urdf::Link> current_link = my_model.getLink(link_names_vector[link_names_vector.size() - 1]);
    std::vector< std::shared_ptr< urdf::Link > > child_link =  current_link->child_links;

    if (child_link.size() == 0) {
      break;
    } else {
      for (std::shared_ptr<urdf::Link> i: child_link) {
	link_names_vector.push_back(i->name);
	visual_vector.push_back(i->visual); // need to get the origin from the visual
      }
    }
  }

  // Print out the list of link names
  for (std::string i: link_names_vector) {
    std::cout << "[" << i << "] ";
  }
  std::cout << std::endl;

  auto kdl_chain = new KDL::Chain();
  std::string base_frame(link_names_vector[0]); // Specify the base to tip you want ie. joint 1 to 2 (base to torso)
  std::string tip_frame(link_names_vector[link_names_vector.size() - 1]);
  if (!my_tree.getChain(base_frame, tip_frame, *kdl_chain)) {
    std::cerr << "not working" << std::endl;
    return;
  }
  mKDLChainSize = kdl_chain->getNrOfSegments();
  std::cout << "The chain has " << mKDLChainSize
	    << " segments" << std::endl
	    << "Found " << link_names_vector.size()
	    << " links" << std::endl;

  std::cout << "This is the joint position array" << std::endl;

  // Initialize the fk solver
  mKDLSolver = new KDL::ChainFkSolverPos_recursive(*kdl_chain);

  // Allocate array for links transformation nodes
  mChainNodeTransforms.resize(mKDLChainSize);

  // Create a vtkMRMLTransform Node for each of these frames
  for (size_t l = 0; l < mKDLChainSize; l++) {
    vtkNew<vtkMRMLTransformStorageNode> storageNode;
    storageNode->SetScene(this->GetMRMLScene());
    vtkNew<vtkMRMLTransformNode> generalTransform;
    generalTransform->SetScene(this->GetMRMLScene());
    mChainNodeTransforms[l] = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
    storageNode->ReadData(mChainNodeTransforms[l].GetPointer());
    mChainNodeTransforms[l]->SetName((link_names_vector[l + 1] + "_transform").c_str());
    this->GetMRMLScene()->AddNode(storageNode.GetPointer());
    this->GetMRMLScene()->AddNode(mChainNodeTransforms[l]);
    mChainNodeTransforms[l]->SetAndObserveStorageNodeID(storageNode->GetID());
  }

  std::vector<double> initialJointValues(mKDLChainSize, 0.0);
  initialJointValues[1] = 0.8; // for testing
  UpdateFK(initialJointValues);

  // Get the origin and rpy
  std::vector<urdf::Pose> origins;
  std::vector<std::string> filenames;
  for (std::shared_ptr<urdf::Visual> i: visual_vector) {
    urdf::Pose origin;
    origin = i->origin;
    origins.push_back(origin);
    // Get stl file name and add it to a list of vectors for python parsing later
    std::shared_ptr<urdf::Mesh> mesh =  std::dynamic_pointer_cast<urdf::Mesh>(i->geometry);
    filenames.push_back(mesh->filename);
  }

  //Call load STL model functions with python - can't find C++ implementation
  QList<QVariant> file_names_for_loading;
  // Link names need to be converted to QList of QVariants to be passed to python script
  for (size_t j = 0; j < filenames.size(); j ++) {
    QVariant file_to_be_added;
    file_to_be_added = QString::fromStdString(filenames[j]);
    file_names_for_loading.append(file_to_be_added);
  }
  //link_names_for_loading.append(link_names_vector);
  #ifdef Slicer_USE_PYTHONQT
    PythonQt::init();
    PythonQtObjectPtr context = PythonQt::self()->getMainModule();
    context.addVariable("fileNamesList", file_names_for_loading);
    context.evalScript(QString(
    "import slicer \n"
    "from pathlib import Path \n"
    "print(fileNamesList) \n"
    "for j in range(len(fileNamesList)): \n"
    " slicer.util.loadModel(str(Path.home()) + fileNamesList[j]) \n" ));
  #endif

  // Set up the initial position for each link (Rotate and Translate based on origin and rpy from the urdf file)
  for (size_t k = 0; k < (mKDLChainSize + 1); k ++) {
    vtkNew<vtkMRMLTransformStorageNode> storageNode;
    vtkSmartPointer<vtkMRMLTransformNode> tnode;
    storageNode->SetScene(this->GetMRMLScene());
    vtkNew<vtkMRMLTransformNode> generalTransform;
    generalTransform->SetScene(this->GetMRMLScene());
    tnode = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
    storageNode->ReadData(tnode.GetPointer());
    tnode->SetName(("InitialPosition_" + link_names_vector[k]).c_str());
    this->GetMRMLScene()->AddNode(storageNode.GetPointer());
    this->GetMRMLScene()->AddNode(tnode);
    tnode->SetAndObserveStorageNodeID(storageNode->GetID());

    vtkTransform * modifiedTransform = vtkTransform::SafeDownCast(tnode->GetTransformToParent());
    urdf::Pose origin = origins[k];
    modifiedTransform->Translate(origin.position.x, origin.position.y, origin.position.z);
    tnode->SetAndObserveTransformToParent(modifiedTransform);
    tnode->Modified();
    vtkTransform * modifiedTransform2 = vtkTransform::SafeDownCast(tnode->GetTransformToParent());
    double r = 0.0;
    double p = 0.0;
    double y = 0.0;
    origin.rotation.getRPY(r, p, y);
    modifiedTransform2->RotateZ(y * (180.0/M_PI)); // RAD to degree conversion - use math.pi instead
    modifiedTransform2->RotateY(p * (180.0/M_PI));
    modifiedTransform2->RotateX(r * (180.0/M_PI));
    tnode->SetAndObserveTransformToParent(modifiedTransform2);
    tnode->Modified();

    vtkNew<vtkMatrix4x4> initialPositionMatrix;
    tnode->GetMatrixTransformToParent(initialPositionMatrix);

    // Apply LPS to RAS conversion
    vtkNew<vtkMatrix4x4> LPSToRAS_matrix;
    LPSToRAS_matrix->SetElement(0, 0, -1.0);
    LPSToRAS_matrix->SetElement(1, 1, -1.0);

    LPSToRAS_matrix->Multiply4x4(initialPositionMatrix, LPSToRAS_matrix, LPSToRAS_matrix);
    tnode->SetMatrixTransformToParent(LPSToRAS_matrix);
    tnode->Modified();

    if (k == 0) {

      vtkMRMLModelNode *modelNode = vtkMRMLModelNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names_vector[k].c_str()));
      assert(modelNode);
      modelNode->SetAndObserveTransformNodeID(tnode->GetID());
    } else {
      vtkMRMLTransformNode *transformNode = vtkMRMLTransformNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName((link_names_vector[k] + "_transform").c_str()));
      tnode->SetAndObserveTransformNodeID(transformNode->GetID());

      // Uncomment for cascaded transforms
      // if (k > 1){
      //   vtkMRMLTransformNode *previousTransformNode = vtkMRMLTransformNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(("InitialPosition_" + link_names_vector[k - 1]).c_str()));
      //   transformNode->SetAndObserveTransformNodeID(previousTransformNode->GetID());
      // }

      vtkMRMLModelNode *modelNode = vtkMRMLModelNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names_vector[k].c_str()));
      modelNode->SetAndObserveTransformNodeID(tnode->GetID());
    }
  }
}

void vtkSlicerRos2Logic::UpdateFK(const std::vector<double> & jointValues)
{
  // make sure the solver exists
  if (!mKDLSolver) {
    std::cout << "FK solver not initialized" << std::endl;
    return;
  }
  // make sure number of joint values is correct
  if (jointValues.size() != mKDLChainSize) {
    std::cout << "FK solver expects " << mKDLChainSize
	      << " values but UpdateFK was called with a vector of size "
	      << jointValues.size() << std::endl;
    return;
  }

  // Set up an std vector of frames
  std::vector<KDL::Frame> FK_frames;
  FK_frames.resize(mKDLChainSize);

  // Convert std::vector to KDL joint array
  auto jointArray = KDL::JntArray(mKDLChainSize);
  for (size_t index = 0; index < mKDLChainSize; ++index) {
    jointArray(index) = jointValues[index];
  }
  std::cout << jointArray(0) << std::endl;
  std::cout << jointArray(1) << std::endl;
  std::cout << jointArray(2) << std::endl;
  std::cout << jointArray(3) << std::endl;
  std::cout << jointArray(4) << std::endl;
  // Calculate forward position kinematics
  mKDLSolver->JntToCart(jointArray, FK_frames);

  //Get the matrix and update it based on the forward kinematics
  for (size_t l = 0; l < mKDLChainSize; l++) {
    KDL::Frame cartpos;
    cartpos = FK_frames[l];
    vtkNew<vtkMatrix4x4> matrix;
    for (size_t i = 0; i < 4; i++) {
      for (size_t j = 0; j < 4; j ++) {
        matrix->SetElement(i, j, cartpos(i, j));
      }
    }
    // Update the matrix for the transform
    mChainNodeTransforms[l]->SetMatrixTransformToParent(matrix);
    mChainNodeTransforms[l]->Modified();
  }
}

void vtkSlicerRos2Logic::Spin(void)
{
  // Spin ROS loop
  rclcpp::spin_some(mNodePointer);
  //queryTfNode(); // COMMENT THIS OUT TO SWTICH BACK TO FK
}

void vtkSlicerRos2Logic::ParameterCallback(std::shared_future<std::vector<rclcpp::Parameter>> future)
{
  parameterNodeCallbackFlag = true;
  auto result = future.get();
  auto param = result.at(0);
  robot_description_string = param.as_string().c_str();

}

void vtkSlicerRos2Logic::JointStateCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg)
{
  std::cerr << "got message of size " << msg->position.size() << std::endl;
  if (msg->position.size() == 6) {
    std::cerr << "commenting out for testing" << std::endl;
    UpdateFK(msg->position); // COMMENT THIS OUT TO SWITCH TO TF
  }
}


void vtkSlicerRos2Logic::Clear()
{
  this->GetMRMLScene()->Clear();
}


void vtkSlicerRos2Logic::queryTfNode()
{
  std::vector<std::string> link_names; // just make the one in intializer a global var
  link_names.push_back("base");
  link_names.push_back("torso");
  link_names.push_back("upper_arm");
  link_names.push_back("lower_arm");
  link_names.push_back("wrist");
  link_names.push_back("tip");
  link_names.push_back("stylus");
  // for (int link = 0; link < link_names.size() - 1; link++) {
  for (int link = 1; link < link_names.size(); link++) {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
      transformStamped = mTfBuffer->lookupTransform(link_names[link], link_names[0], tf2::TimePointZero);
      std::cout << "Recieved transform" << link << std::endl;
      updateTransformFromTf(transformStamped, link - 1);
      } catch (tf2::TransformException & ex) {
        std::cout << " Transform exception" << std::endl;
        return;
    }

  }
}

void vtkSlicerRos2Logic::updateTransformFromTf(geometry_msgs::msg::TransformStamped transformStamped, int transform)
{
  // Retrieve the translation vector and quaternion from the geometry message

  auto x = transformStamped.transform.translation.x;
  auto y = transformStamped.transform.translation.y;
  auto z = transformStamped.transform.translation.z; // convert from m to mm
  auto q_w = transformStamped.transform.rotation.w;
  auto q_x = transformStamped.transform.rotation.x;
  auto q_y = transformStamped.transform.rotation.y;
  auto q_z = transformStamped.transform.rotation.z;
  std::cerr << "Got transform" << std::endl;
  std::cerr << "x: " << x << "y: " << y << "z: " << z << "w: " << q_w << "x: " <<  q_x << "y: " << q_y << "z: " << q_z << std::endl;
  // Right now this rotates the torso according to a defined transform from the state_publisher - if we write all the transforms then we would have to define the header for each and rotate
  // that node accordingly in the mNodeTransforms list - so basically the logic works now but we have to work on publisher to use it

  if (mKDLChainSize > 0){ // Make sure the KDL chain is defined to avoid crash
    vtkTransform * modifiedTransform2 = vtkTransform::SafeDownCast(mChainNodeTransforms[transform]->GetTransformToParent());
    modifiedTransform2->RotateWXYZ(q_w, q_x, q_y, q_z);
    //modifiedTransform2->Translate(x, y, z); // Its a rotational joint so we shouldn't need translation right?
    modifiedTransform2->Modified();
    //mChainNodeTransforms[transform]->SetAndObserveTransformToParent(modifiedTransform2);
    //mChainNodeTransforms[transform]->Modified();

    vtkNew<vtkMatrix4x4> t;
    // //mChainNodeTransforms[transform]->GetMatrixTransformToParent(t);
    //
    modifiedTransform2->GetMatrix(t);
    // // Apply LPS to RAS conversion
    // // x forward, y left, z up
    // // a forward, r left, s up
    vtkNew<vtkMatrix4x4> RosToRAS_matrix;
    RosToRAS_matrix->SetElement(0, 1, 1.0);
    RosToRAS_matrix->SetElement(0, 0, 0.0);
    RosToRAS_matrix->SetElement(1, 1, 0.0);
    RosToRAS_matrix->SetElement(1, 0, 1.0);
    //
    RosToRAS_matrix->Multiply4x4(t, RosToRAS_matrix, t);
    mChainNodeTransforms[transform]->SetMatrixTransformToParent(t);
    mChainNodeTransforms[transform]->Modified();

  }
}
