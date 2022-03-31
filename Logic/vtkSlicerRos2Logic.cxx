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
#include <ament_index_cpp/get_package_share_directory.hpp>

// SlicerRos2 Logic includes
#include "vtkSlicerRos2Logic.h"

// MRML includes
#include <vtkMRMLScene.h>
#include <vtkMRMLModelNode.h>
#include <vtkMRMLTransformStorageNode.h>
#include <vtkMRMLTransformNode.h>
#include <vtkMRMLLinearTransformNode.h>
#include <vtkMRMLModelStorageNode.h>

// VTK includes
#include <vtkMatrix4x4.h>
#include <vtkMatrix3x3.h>
#include <vtkMath.h>
#include <vtkQuaternion.h>
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
// #include <boost/filesystem/path.hpp>
#include <regex>

// RQt includes
 #include <rqt_gui_cpp/plugin.h>

// Reading file includes
#include <iostream>
#include <fstream>
#include <sstream>

using std::ifstream; using std::ostringstream; using std::string;

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

  // create the ROS node
  mNodePointer = std::make_shared<rclcpp::Node>(nodeName);

  // Set up the listener in the constructor because this is the default behaviour
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

void vtkSlicerRos2Logic::SetModelNodeAndParameter(const std::string & nodeName,
						  const std::string & parameterName)
{
  // re-initialize model variables
  mModel.Loaded = false;
  mModel.ComesFromFile = false;
  mModel.Parameter.NodeName = nodeName;
  mModel.Parameter.NodeFound = false;
  mModel.Parameter.ParameterName = parameterName;
  mModel.Parameter.ParameterFound = false;

  // create parameter client
  mParameterClient
    = std::make_shared<rclcpp::AsyncParametersClient>
    (mNodePointer,
     mModel.Parameter.NodeName);

  std::chrono::seconds sec(1);
  mParameterClient->wait_for_service(sec);
  auto parameters_future
    = mParameterClient->get_parameters
    ({mModel.Parameter.ParameterName},
     std::bind(&vtkSlicerRos2Logic::ModelParameterCallback,
               this, std::placeholders::_1));
}

//---------------------------------------------------------------------------
void vtkSlicerRos2Logic::SetModelFile(const std::string & selectedFile){

  // re-initialize model variables
  mModel.Loaded = false;
  mModel.ComesFromFile = true;

  ifstream input_file(selectedFile);
  if (!input_file.is_open()){
    std::cerr << "Couldn't open file" << std::endl;
  }
  else {
    mModel.URDF = string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());
    loadRobotSTLModels();
  }

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

  // Print out the urdf from param
  std::cout << mModel.URDF << std::endl;
  // Parser the urdf file into an urdf model - to get names of links and pos/ rpy
  urdf::Model my_model;
  if (!my_model.initString(mModel.URDF)) {
      return;
  }

  // load urdf file into a kdl tree to do forward kinematics
  // KDL::Tree my_tree;
  // if (!kdl_parser::treeFromString(mModel.URDF, my_tree)) {
  //   return;
  // }

  // Start by getting the name of the root link and add it to the vector of strings
  std::shared_ptr<const urdf::Link> root = my_model.getRoot();
  std::string root_name = root->name;
  link_names_vector.push_back(root_name);
  std::vector< std::shared_ptr< urdf::Visual > > visual_vector;
  visual_vector.push_back(root->visual);

  size_t lastExplored = 0;
  while (lastExplored != visual_vector.size()){
    std::shared_ptr<const urdf::Link> current_link = my_model.getLink(link_names_vector[lastExplored]);
    std::vector< std::shared_ptr< urdf::Link > > child_link =  current_link->child_links;
    for (std::shared_ptr<urdf::Link> i: child_link) {
    	link_names_vector.push_back(i->name);
    	visual_vector.push_back(i->visual); // need to get the origin from the visual
    }
    lastExplored++;
  }

  // while (true) {
  //   std::shared_ptr<const urdf::Link> current_link = my_model.getLink(link_names_vector[link_names_vector.size() - 1]);
  //   std::vector< std::shared_ptr< urdf::Link > > child_link =  current_link->child_links;
  //
  //   if (child_link.size() == 0) {
  //     break;
  //   } else {
  //     for (std::shared_ptr<urdf::Link> i: child_link) {
	// link_names_vector.push_back(i->name);
	// visual_vector.push_back(i->visual); // need to get the origin from the visual
  //     }
  //   }
  // }

  // Print out the list of link names
  std::cerr << "Link name list" << std::endl;
  for (std::string i: link_names_vector) {
    std::cout << "[" << i << "] ";
  }
  std::cout << std::endl;

  // auto kdl_chain = new KDL::Chain();
  // std::string base_frame(link_names_vector[0]); // Specify the base to tip you want ie. joint 1 to 2 (base to torso)
  // std::string tip_frame(link_names_vector[link_names_vector.size() - 1]);
  // if (!my_tree.getChain(base_frame, tip_frame, *kdl_chain)) {
  //   std::cerr << "not working" << std::endl;
  //   return;
  // }
  // mKDLChainSize = kdl_chain->getNrOfSegments();
  // std::cout << "The chain has " << mKDLChainSize
	//     << " segments" << std::endl
	//     << "Found " << link_names_vector.size()
	//     << " links" << std::endl;
  //
  // std::cout << "This is the joint position array" << std::endl;
  //
  // // Initialize the fk solver
  // mKDLSolver = new KDL::ChainFkSolverPos_recursive(*kdl_chain);
  //
  // // Allocate array for links transformation nodes
  mKDLChainSize = link_names_vector.size();
  mChainNodeTransforms.resize(mKDLChainSize);
  std::cerr << "Length of link names vector" << link_names_vector.size() << std::endl;

  // Create a vtkMRMLTransform Node for each of these frames
#if 0
  for (size_t l = 1; l < mKDLChainSize; l++) {
    vtkNew<vtkMRMLTransformStorageNode> storageNode;
    storageNode->SetScene(this->GetMRMLScene());
    vtkNew<vtkMRMLTransformNode> generalTransform;
    generalTransform->SetScene(this->GetMRMLScene());
    mChainNodeTransforms[l] = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
    storageNode->ReadData(mChainNodeTransforms[l].GetPointer());
    mChainNodeTransforms[l]->SetName((link_names_vector[l] + "_transform").c_str());
    this->GetMRMLScene()->AddNode(storageNode.GetPointer());
    this->GetMRMLScene()->AddNode(mChainNodeTransforms[l]);
    mChainNodeTransforms[l]->SetAndObserveStorageNodeID(storageNode->GetID());
  }
#endif

  // std::vector<double> initialJointValues(mKDLChainSize, 0.0);
  // initialJointValues[1] = 0.8; // for testing
  //UpdateFK(initialJointValues);
  std::cerr << "Getting past transform declartion" << std::endl;
  // Get the origin and rpy
  std::vector<urdf::Pose> origins;
  std::vector<std::string> filenames;
  filenames.resize(visual_vector.size());
  for (size_t index = 0; index < visual_vector.size(); ++index) {
    std::shared_ptr<urdf::Visual> i = visual_vector[index];
    if (i == nullptr) {
      std::cerr << "no visual" << std::endl;
    } else {
      urdf::Pose origin;
      origin = i->origin;
      origins.push_back(origin);
      // Get stl file name and add it to a list of vectors for python parsing later
      std::shared_ptr<urdf::Mesh> mesh =  std::dynamic_pointer_cast<urdf::Mesh>(i->geometry);
      if (mesh != nullptr) {
        // See if the file name uses a package url
        std::string filename = mesh->filename;
        std::cerr << index << ": " << filename << std::endl;
        std::regex param_regex("^package:\\/\\/(\\w+)\\/(.*)");
        std::smatch match;
        if (std::regex_search(filename, match, param_regex)) {
          const std::string package = match[1];
          const std::string relativeFile = match[2];
          // Anton: add try/catch here in case the package is not found!
          const std::string packageShareDirectory
             = ament_index_cpp::get_package_share_directory(package);
          filename = packageShareDirectory + "/" + relativeFile;
        }
        filenames[index] = filename;
        std::cerr << index << ": " << filename << std::endl;
      } else {
        std::cerr << "link " << index << " has a visual, but not from file" << std::endl;
      }
    }
  }

  std::cerr << "Getting past file name fetch" << std::endl;
  //Call load STL model functions with python - can't find C++ implementation
  #if 0
  QList<QVariant> file_names_for_loading;
  // Link names need to be converted to QList of QVariants to be passed to python script
  for (size_t j = 0; j < filenames.size(); j ++) {
    if (!filenames[j].empty()) {
      QVariant file_to_be_added;
      file_to_be_added = QString::fromStdString(filenames[j]);
      file_names_for_loading.append(file_to_be_added);
    }
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
    " slicer.util.loadModel(fileNamesList[j]) \n" ));
  #endif
  #endif

  std::cerr << "Getting past python load" << std::endl;
  // Set up the initial position for each link (Rotate and Translate based on origin and rpy from the urdf file)
  for (size_t k = 0; k < (mKDLChainSize); k ++) {

    // this was in loop before python
    vtkNew<vtkMRMLTransformStorageNode> storageNode1;
    storageNode1->SetScene(this->GetMRMLScene());
    //vtkNew<vtkMRMLTransformNode> generalTransform1;
    //generalTransform1->SetScene(this->GetMRMLScene());
    mChainNodeTransforms[k] = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
    storageNode1->ReadData(mChainNodeTransforms[k].GetPointer());
    mChainNodeTransforms[k]->SetName((link_names_vector[k] + "_transform").c_str());
    this->GetMRMLScene()->AddNode(storageNode1.GetPointer());
    this->GetMRMLScene()->AddNode(mChainNodeTransforms[k]);
    mChainNodeTransforms[k]->SetAndObserveStorageNodeID(storageNode1->GetID());


    std::cerr << k << " a" << std::endl;
    vtkNew<vtkMRMLTransformStorageNode> storageNode;
    vtkSmartPointer<vtkMRMLTransformNode> tnode;
    storageNode->SetScene(this->GetMRMLScene());
    //vtkNew<vtkMRMLTransformNode> generalTransform;
    //generalTransform->SetScene(this->GetMRMLScene());
    tnode = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
    storageNode->ReadData(tnode.GetPointer());
    tnode->SetName(("InitialPosition_" + link_names_vector[k]).c_str());
    this->GetMRMLScene()->AddNode(storageNode.GetPointer());
    this->GetMRMLScene()->AddNode(tnode);
    tnode->SetAndObserveStorageNodeID(storageNode->GetID());
std::cerr << k << " b" << std::endl;
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
std::cerr << k << " c" << std::endl;
    vtkNew<vtkMatrix4x4> initialPositionMatrix;
    tnode->GetMatrixTransformToParent(initialPositionMatrix);

    // Apply LPS to RAS conversion
    vtkNew<vtkMatrix4x4> LPSToRAS_matrix;
    LPSToRAS_matrix->SetElement(0, 0, -1.0);
    LPSToRAS_matrix->SetElement(1, 1, -1.0);

    LPSToRAS_matrix->Multiply4x4(initialPositionMatrix, LPSToRAS_matrix, initialPositionMatrix);
    tnode->SetMatrixTransformToParent(initialPositionMatrix);
    tnode->Modified();
std::cerr << k << " d" << std::endl;
    // if (k == 0) {
    //   vtkMRMLModelNode *modelNode = vtkMRMLModelNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names_vector[k].c_str()));
    //   assert(modelNode);
    //   modelNode->SetAndObserveTransformNodeID(tnode->GetID());
    // std::cerr << k << " d1" << std::endl;
    // } else {


      // vtkMRMLTransformNode *transformNode = vtkMRMLTransformNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName((link_names_vector[k] + "_transform").c_str()));
      tnode->SetAndObserveTransformNodeID(mChainNodeTransforms[k]->GetID()); // transformNode->GetID());

      if (mModel.Serial == true){
        if (k > 1){
          // vtkMRMLTransformNode *previousTransformNode = vtkMRMLTransformNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName((link_names_vector[k - 1] + "_transform").c_str()));
          // transformNode->SetAndObserveTransformNodeID(previousTransformNode->GetID());
	  // mChainNodeTransforms[k]->SetAndObserveTransformNodeID(previousTransformNode->GetID());
	  mChainNodeTransforms[k]->SetAndObserveTransformNodeID(mChainNodeTransforms[k-1]->GetID());
        }
      }

      // vtkMRMLModelNode *modelNode = vtkMRMLModelNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName(link_names_vector[k].c_str()));   // for nodes created using Python
      vtkNew<vtkMRMLModelStorageNode> modelStorageNode;
      vtkNew<vtkMRMLModelNode> modelNode;
      modelStorageNode->SetScene(this->GetMRMLScene());
      modelNode->SetScene(this->GetMRMLScene());
      modelNode->SetName((link_names_vector[k] + "_model").c_str());
      if (!(filenames[k].empty())) {
        modelStorageNode->SetFileName((filenames[k]).c_str());
        modelStorageNode->Modified();
        //modelStorageNode->ReadDataInternal(modelNode);
      }


      modelNode->SetAndObserveStorageNodeID(modelStorageNode->GetID());
      modelNode->Modified();
      this->GetMRMLScene()->AddNode(modelStorageNode); //.GetPointer());
      this->GetMRMLScene()->AddNode(modelNode);

      //modelNode->SetAndObserveTransformNodeID(tnode->GetID());
      std::cerr << k << " d2" << std::endl;
    //}
  }
  std::cerr << "------------- Model loaded!" << std::endl;
  mModel.Loaded = true;
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
  if (rclcpp::ok()) {
    rclcpp::spin_some(mNodePointer);
    if (mModel.Loaded && !mRobotState.IsUsingTopic) {
      queryTfNode();
    }
  }
}

void vtkSlicerRos2Logic::ModelParameterCallback(std::shared_future<std::vector<rclcpp::Parameter>> future)
{
  // get the URDF as a single string
  auto result = future.get();
  // Anton: we should make sure there is a result
  auto param = result.at(0);
  mModel.URDF = param.as_string().c_str();
  loadRobotSTLModels();
}

void vtkSlicerRos2Logic::JointStateCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg)
{
  if (mRobotState.IsUsingTopic == true){
    if (msg->position.size() == 6) {
      std::cerr << "No loading" << std::endl;
      //UpdateFK(msg->position);
    }
  }
  else{
    return;
  }

}


void vtkSlicerRos2Logic::Clear()
{
  this->GetMRMLScene()->Clear();
}


void vtkSlicerRos2Logic::queryTfNode()
{

  for (int link = 1; link < link_names_vector.size(); link++) {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = mTfBuffer->lookupTransform(link_names_vector[link - 1], link_names_vector[link], tf2::TimePointZero);
      updateTransformFromTf(transformStamped, link - 1);
      } catch (tf2::TransformException & ex) {
        std::cout << " Transform exception" << std::endl;
        return;
    }

  }
  mChainNodeTransforms[0]->Modified();
}

void vtkSlicerRos2Logic::updateTransformFromTf(geometry_msgs::msg::TransformStamped transformStamped, int transform)
{
  // Retrieve the translation vector and quaternion from the geometry message
  auto x = transformStamped.transform.translation.x;
  auto y = transformStamped.transform.translation.y;
  auto z = transformStamped.transform.translation.z;
  auto q_w = transformStamped.transform.rotation.w;
  auto q_x = transformStamped.transform.rotation.x;
  auto q_y = transformStamped.transform.rotation.y;
  auto q_z = transformStamped.transform.rotation.z;


  if (mKDLChainSize > 0){ // Make sure the KDL chain is defined to avoid crash
    const double q[4] = {q_w, q_x, q_y, q_z};
    double A[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};
    vtkMath::QuaternionToMatrix3x3(q, A); // Convert quaternion to a 3x3 matrix
    vtkNew<vtkMatrix4x4> Tf;
    for (size_t row = 0; row < 3; row++) {
      for (size_t column = 0; column < 3; column++) {
        Tf->SetElement(row, column, A[row][column]); // Set the 3x3 matrix as the rotation component of the homogeneous transform
      }
    }
    // Apply translation vector
    Tf->SetElement(0,3, x);
    Tf->SetElement(1,3, y);
    Tf->SetElement(2,3, z);

    mChainNodeTransforms[transform]->SetMatrixTransformToParent(Tf);
    //mChainNodeTransforms[transform]->Modified();
  }
}

void vtkSlicerRos2Logic::SetRobotStateTopic(const std::string & topicName){

  mRobotState.IsUsingTopic = true;
  // subscription
  mJointStateSubscription
    = mNodePointer->create_subscription<sensor_msgs::msg::JointState>
    (topicName, 10, std::bind(&vtkSlicerRos2Logic::JointStateCallback,
				  this, std::placeholders::_1));
  mModel.Serial = false;
  // Should this clean up the Tf listener?
}

void vtkSlicerRos2Logic::SetRobotStateTf(){

  mRobotState.IsUsingTopic = false;
  mModel.Serial = true;

  // Remove the subscription
  if (mJointStateSubscription != NULL){
    mJointStateSubscription.reset();
  }

}
