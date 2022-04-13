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
#include <vtkMRMLDisplayNode.h>
#include <vtkMRMLModelDisplayNode.h>

// VTK includes
#include <vtkMatrix4x4.h>
#include <vtkMatrix3x3.h>
#include <vtkMath.h>
#include <vtkQuaternion.h>
#include <vtkTransform.h>
#include <vtkSTLReader.h>
#include <vtkOBJReader.h>
#include <vtkPointSet.h>
#include <vtkUnstructuredGrid.h>
#include <vtkPolyData.h>

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
 #include <tf2_ros/transform_broadcaster.h>

// Reading file includes
#include <iostream>
#include <fstream>
#include <sstream>

// Qt includes
#include <QDialog>
#include <QLabel>
#include <QVBoxLayout>

using std::ifstream; using std::ostringstream; using std::string;
auto const MM_TO_M_CONVERSION = 1000.00;

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

  mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*mNodePointer);

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

  // Start by getting the name of the root link and add it to the vector of strings
  std::shared_ptr<const urdf::Link> root = my_model.getRoot();
  std::string root_name = root->name;
  link_names_vector.push_back(root_name);
  link_parent_names_vector.push_back(root_name);
  std::vector< std::shared_ptr< urdf::Visual > > visual_vector;
  visual_vector.push_back(root->visual);
  size_t lastExplored = 0;
  while (lastExplored != visual_vector.size()){
    std::shared_ptr<const urdf::Link> current_link = my_model.getLink(link_names_vector[lastExplored]);
    std::vector< std::shared_ptr< urdf::Link > > child_link =  current_link->child_links;
    // Check if the model is Serial
    if (child_link.size() > 1){
      mModel.Serial = false;
    }

    for (std::shared_ptr<urdf::Link> i: child_link) {
    	link_names_vector.push_back(i->name);
      link_parent_names_vector.push_back(current_link->name);
      std::string child_name = i->name;
    	visual_vector.push_back(i->visual); // need to get the origin from the visual
    }
    lastExplored++;
  }

  // // Allocate array for links transformation nodes
  //mKDLChainSize = link_names_vector.size();
  //mChainNodeTransforms.resize(mKDLChainSize);

  if (mRobotState.IsUsingTopic){
    if (!mModel.Serial){
      QDialog *popupDialog = new QDialog();
      QLabel *label = new QLabel();
      label->setText("Topic transforms are not supported for parallel manipulators.");
      QVBoxLayout *layout = new QVBoxLayout();
      layout->addWidget(label);
      popupDialog->setLayout(layout);
      popupDialog->show();
      return;
    }
    initializeFkSolver();
    mChainNodeTransforms.resize(mKDLChainSize + 1);
  }
  else {
    mKDLChainSize = link_names_vector.size();
    mChainNodeTransforms.resize(mKDLChainSize);
  }


  // Get the origin and rpy
  std::vector<urdf::Pose> origins;
  std::vector<std::string> filenames;
  filenames.resize(visual_vector.size());
  origins.resize(visual_vector.size());
  // This was causing a big issue before - dvrk showing up in the wrong places
  for (size_t index = 0; index < visual_vector.size(); ++index) {
    std::shared_ptr<urdf::Visual> i = visual_vector[index];
    if (i == nullptr) {
      std::cerr << "no visual" << std::endl;
    } else {
      urdf::Pose origin;
      origin = i->origin;
      origins[index] = origin;
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

  // This is  hack for the FK/ Tf issue - need to figure out how to write this so it's intuitive to read
  int counter;
  if (mRobotState.IsUsingTopic){
    counter = mKDLChainSize + 1;
  }
  else{
    counter = mKDLChainSize;
  }

  // Set up the initial position for each link (Rotate and Translate based on origin and rpy from the urdf file)
  for (size_t k = 0; k < (counter); k ++) {

    vtkNew<vtkMRMLTransformStorageNode> storageNode1;
    storageNode1->SetScene(this->GetMRMLScene());
    mChainNodeTransforms[k] = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
    storageNode1->ReadData(mChainNodeTransforms[k].GetPointer());
    mChainNodeTransforms[k]->SetName((link_names_vector[k] + "_transform").c_str());
    this->GetMRMLScene()->AddNode(storageNode1.GetPointer());
    this->GetMRMLScene()->AddNode(mChainNodeTransforms[k]);
    mChainNodeTransforms[k]->SetAndObserveStorageNodeID(storageNode1->GetID());

    vtkNew<vtkMRMLTransformStorageNode> storageNode;
    vtkSmartPointer<vtkMRMLTransformNode> tnode;
    storageNode->SetScene(this->GetMRMLScene());
    tnode = vtkSmartPointer<vtkMRMLTransformNode>::Take(vtkMRMLLinearTransformNode::New());
    storageNode->ReadData(tnode.GetPointer());
    tnode->SetName(("InitialPosition_" + link_names_vector[k]).c_str());
    this->GetMRMLScene()->AddNode(storageNode.GetPointer());
    this->GetMRMLScene()->AddNode(tnode);
    tnode->SetAndObserveStorageNodeID(storageNode->GetID());
    vtkTransform * modifiedTransform = vtkTransform::SafeDownCast(tnode->GetTransformToParent());
    urdf::Pose origin = origins[k];
    modifiedTransform->Translate(origin.position.x*MM_TO_M_CONVERSION, origin.position.y*MM_TO_M_CONVERSION, origin.position.z*MM_TO_M_CONVERSION);
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

    // Scale everything up - account for unit conversion from (mm to m)
    vtkNew<vtkMatrix4x4> MmToM_Transform;
    MmToM_Transform->SetElement(0, 0, MM_TO_M_CONVERSION);
    MmToM_Transform->SetElement(1, 1, MM_TO_M_CONVERSION);
    MmToM_Transform->SetElement(2, 2, MM_TO_M_CONVERSION);
    MmToM_Transform->Multiply4x4(initialPositionMatrix, MmToM_Transform, initialPositionMatrix);

    tnode->SetMatrixTransformToParent(initialPositionMatrix);
    tnode->Modified();


      // vtkMRMLTransformNode *transformNode = vtkMRMLTransformNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName((link_names_vector[k] + "_transform").c_str()));
      tnode->SetAndObserveTransformNodeID(mChainNodeTransforms[k]->GetID()); // transformNode->GetID());

      if (!mRobotState.IsUsingTopic){
        if (link_parent_names_vector[k] != "world"){
          vtkMRMLTransformNode *parentTransformNode = vtkMRMLTransformNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName((link_parent_names_vector[k] + "_transform").c_str()));
  	        mChainNodeTransforms[k]->SetAndObserveTransformNodeID(parentTransformNode->GetID());
          }
        }

      // Read the STL file and add the model to the scene - set the name to be the link name instead of file name
      // Note this code is a repeat of function implemented in vtkMRMLModelStorageNode - in Slicer MRML core - AddFileName should hopefully do the same and save efficiency (modelStorageNode->SetFileName((filenames[k]).c_str());)
      if (!filenames[k].empty()){
        if (filenames[k].find(".stl") or filenames[k].find(".STL")){
          std::cerr << "Model file format is STL" << std::endl;
          vtkNew<vtkSTLReader> reader; // default is STL
          reader->SetFileName(filenames[k].c_str());
          reader->Update();
          vtkSmartPointer<vtkPointSet> meshFromFile;
          meshFromFile = reader->GetOutput();
          vtkSmartPointer<vtkPointSet> meshToSetInNode;
          meshToSetInNode = meshFromFile;
          vtkNew< vtkMRMLModelNode > modelNode;
          this->GetMRMLScene()->AddNode( modelNode.GetPointer() );
          modelNode->SetName((link_names_vector[k] + "_model").c_str());
          modelNode->SetAndObserveMesh(meshToSetInNode);
          // Create display node
          if (modelNode->GetDisplayNode() == NULL){
              vtkNew< vtkMRMLModelDisplayNode > displayNode;
              this->GetMRMLScene()->AddNode( displayNode.GetPointer() );
              displayNode->SetName((link_names_vector[k] + "_model_display_node").c_str());
              modelNode->SetAndObserveDisplayNodeID( displayNode->GetID() );
            }
          // Set the model node to listen to the right transform
          modelNode->SetAndObserveTransformNodeID(tnode->GetID());
        }
        if(filenames[k].find(".obj") or filenames[k].find(".OBJ")){
          std::cerr << "Model file format is OBJ" << std::endl;
          vtkNew<vtkOBJReader> reader; // This is not updating for some reason
          reader->SetFileName(filenames[k].c_str());
          reader->Update();
          vtkSmartPointer<vtkPointSet> meshFromFile;
          meshFromFile = reader->GetOutput();
          vtkSmartPointer<vtkPointSet> meshToSetInNode;
          meshToSetInNode = meshFromFile;
          vtkNew< vtkMRMLModelNode > modelNode;
          this->GetMRMLScene()->AddNode( modelNode.GetPointer() );
          modelNode->SetName((link_names_vector[k] + "_model").c_str());
          modelNode->SetAndObserveMesh(meshToSetInNode);
          // Create display node
          if (modelNode->GetDisplayNode() == NULL){
              vtkNew< vtkMRMLModelDisplayNode > displayNode;
              this->GetMRMLScene()->AddNode( displayNode.GetPointer() );
              displayNode->SetName((link_names_vector[k] + "_model_display_node").c_str());
              modelNode->SetAndObserveDisplayNodeID( displayNode->GetID() );
            }
          // Set the model node to listen to the right transform
          modelNode->SetAndObserveTransformNodeID(tnode->GetID());
        }
        else{
          std::cerr << "Model file type is not supported" << std::endl;
          return;
        }
      }
  }
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
  // TODO: Figure out why the indexing needs to be like this and where the stylus went
  for (size_t l = 0; l < (mKDLChainSize); l++) { // TODO: This shoul be the size of mChainNodeTransforms
    KDL::Frame cartpos;
    cartpos = FK_frames[l];
    vtkNew<vtkMatrix4x4> matrix;
    for (size_t i = 0; i < 4; i++) {
      for (size_t j = 0; j < 4; j ++) {
        if (i == 0 & j == 3){
          matrix->SetElement(i, j, cartpos(i, j)*MM_TO_M_CONVERSION);
        }
        else if (i == 1 & j == 3){
          matrix->SetElement(i, j, cartpos(i, j)*MM_TO_M_CONVERSION);
        }
        else if (i == 2 & j == 3){
          matrix->SetElement(i, j, cartpos(i, j)*MM_TO_M_CONVERSION);
        }
        else {
          matrix->SetElement(i, j, cartpos(i, j));
        }
      }
    }

    mChainNodeTransforms[l + 1]->SetMatrixTransformToParent(matrix);
    mChainNodeTransforms[l + 1]->Modified();
  }
}

void vtkSlicerRos2Logic::Spin(void)
{
  // Spin ROS loop
  if (rclcpp::ok()) {
    rclcpp::spin_some(mNodePointer);
    if (mModel.Loaded && !mRobotState.sendingTf && !mRobotState.IsUsingTopic) {
      queryTfNode();
    }
    else if (mModel.Loaded && mRobotState.sendingTf && !mRobotState.IsUsingTopic){
      BroadcastTransform();
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
      UpdateFK(msg->position);
    }
  }
  else{
    return;
  }

}


void vtkSlicerRos2Logic::Clear()
{
  this->GetMRMLScene()->Clear();

  mKDLSolver = 0;
  mKDLChainSize = 0;
  parameterNodeCallbackFlag = false;
  
  // Need to handle mChainNodeTransforms
  mParameterClient.reset();
  link_names_vector.clear();
  link_parent_names_vector.clear();

  // Restore defaults
  // Robot state
  mRobotState.IsUsingTopic = false;
  mRobotState.sendingTf = false;
  mRobotState.Topic = "";

  // Robot model
  mModel.Loaded = false;
  mModel.URDF = "";
  mModel.ComesFromFile = false;
  mModel.FileName = "";
  mModel.Serial = true;
  mModel.Parameter.NodeName = "";
  mModel.Parameter.ParameterName = "";
  mModel.Parameter.NodeFound = false;
  mModel.Parameter.ParameterFound = false;


}


void vtkSlicerRos2Logic::queryTfNode()
{
  for (int link = 0; link < link_names_vector.size(); link++) {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      if (link == 0) {
        // Probably don't need this if - the parent of the first link should be just world
        transformStamped = mTfBuffer->lookupTransform(link_names_vector[link], link_names_vector[link], tf2::TimePointZero);
      } else {
        transformStamped = mTfBuffer->lookupTransform(link_parent_names_vector[link], link_names_vector[link], tf2::TimePointZero);
      }
      updateTransformFromTf(transformStamped, link);
    } catch (tf2::TransformException & ex) {
      std::cout << " Transform exception" << std::endl;
    }
  }
  //mChainNodeTransforms[0]->Modified();
}

void vtkSlicerRos2Logic::updateTransformFromTf(geometry_msgs::msg::TransformStamped transformStamped, int transform)
{
  // Retrieve the translation vector and quaternion from the geometry message
  auto x = transformStamped.transform.translation.x*MM_TO_M_CONVERSION;
  auto y = transformStamped.transform.translation.y*MM_TO_M_CONVERSION;
  auto z = transformStamped.transform.translation.z*MM_TO_M_CONVERSION;
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
    mChainNodeTransforms[transform]->Modified();
  }
}

void vtkSlicerRos2Logic::SetRobotStateTopic(const std::string & topicName){

  mRobotState.IsUsingTopic = true;
  // subscription
  mJointStateSubscription
    = mNodePointer->create_subscription<sensor_msgs::msg::JointState>
    (topicName, 10, std::bind(&vtkSlicerRos2Logic::JointStateCallback,
				  this, std::placeholders::_1));
}

void vtkSlicerRos2Logic::SetRobotStateTf(){

  mRobotState.IsUsingTopic = false;
  // Remove the subscription
  if (mJointStateSubscription != NULL){
    mJointStateSubscription.reset();
  }

}

void vtkSlicerRos2Logic::BroadcastTransform(){

  mRobotState.sendingTf = true;
  std::string parent;
  for (int link = 0; link < link_names_vector.size(); link++) {
    if (link_names_vector[link] == link_parent_names_vector[link]){
      parent = "world";
    }
    else{
      parent = link_parent_names_vector[link];
    }
      // This should get the transform from 3D Slicer - try to upgrade it based on transform in 3D Slicer
    vtkMRMLTransformNode *transformNode = vtkMRMLTransformNode::SafeDownCast(this->GetMRMLScene()->GetFirstNodeByName((link_names_vector[link] + "_transform").c_str()));
    float pos[3] = {0.0, 0.0, 0.0}; // translation vector
    double q[4] = {0.0, 0.0, 0.0, 0.0}; // quaternion vector
    double A[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}}; // 3x3 rotation matrix

    vtkNew<vtkMatrix4x4> matrix;
    transformNode->GetMatrixTransformToParent(matrix);
    for (int i = 0; i < 3; i ++){
      for (int j = 0; j < 3; j ++){
        A[i][j] =  matrix->GetElement(i, j);
      }
    }
    //
    vtkMath::Matrix3x3ToQuaternion(A, q); // Convert quaternion to a 3x3 matrix
    pos[0] = matrix->GetElement(0,3); // Get the translation vector from the homogeneous transformation matrix
    pos[1] = matrix->GetElement(1,3);
    pos[2] = matrix->GetElement(2,3);

    geometry_msgs::msg::TransformStamped transformStamped;
    rclcpp::Time now = mNodePointer->get_clock()->now();
    transformStamped.header.stamp = now;
    transformStamped.header.frame_id = parent; //"torso"; // should have header be torso and child be base
    transformStamped.child_frame_id =  link_names_vector[link];// if you swap this and do it incorectly you can confirm that we're updating tf because the rviz updates to - just need to figure out how to do the dot example
    transformStamped.transform.translation.x = pos[0]/MM_TO_M_CONVERSION;
    transformStamped.transform.translation.y = pos[1]/MM_TO_M_CONVERSION;
    transformStamped.transform.translation.z = pos[2]/MM_TO_M_CONVERSION;
    transformStamped.transform.rotation.w = q[0];
    transformStamped.transform.rotation.x = q[1];
    transformStamped.transform.rotation.y = q[2];
    transformStamped.transform.rotation.z = q[3];

    // Send the transform
    mTfBroadcaster->sendTransform(transformStamped);
  }
}

void vtkSlicerRos2Logic::initializeFkSolver(){

  KDL::Tree my_tree;
  if (!kdl_parser::treeFromString(mModel.URDF, my_tree)) {
    return;
  }

  auto kdl_chain = new KDL::Chain();

  std::string base_frame(link_names_vector[0]); // Specify the base to tip you want ie. joint 1 to 2 (base to torso)
  std::string tip_frame(link_names_vector[link_names_vector.size() - 1]);
  std::cerr << "Tip frame: " << tip_frame << std::endl;
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
}
