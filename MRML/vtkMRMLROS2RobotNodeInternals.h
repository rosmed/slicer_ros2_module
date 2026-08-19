#ifndef __vtkMRMLROS2RobotNodeInternals_h
#define __vtkMRMLROS2RobotNodeInternals_h

// urdf
#include <urdf/model.h>

// MoveIt kinematics and planning includes
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// KDL includes
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <vtkMRMLModelNode.h>
#include <vtkMRMLROS2Tf2LookupNode.h>
#include <vtkSmartPointer.h>

class vtkMRMLROS2RobotNodeInternals
{

 public:

  virtual ~vtkMRMLROS2RobotNodeInternals() = default;
  urdf::Model mURDFModel;
  std::vector< std::shared_ptr< urdf::Visual > > mVisualVector;
  std::map< std::string, std::shared_ptr< urdf::Material > > mMaterialsMap;
  std::vector< std::string> mLinkMaterials;
  std::vector< std::shared_ptr< urdf::Material > > mMaterialVector;
  std::shared_ptr<const urdf::Link> mParentLinkPointer;
  std::vector< std::shared_ptr< urdf::Link > > mChildLinkPointer;
  std::vector<urdf::Pose> mLinkOrigins;

  // Transient setup data
  std::vector<std::string> mLinkNames;
  std::vector<std::string> mLinkParentNames;
  std::vector<std::string> mLinkModelFiles;
  std::vector< std::vector< vtkSmartPointer< vtkMRMLModelNode > > > mLinkModels;
  std::vector< vtkSmartPointer< vtkMRMLROS2Tf2LookupNode > > mLookupNodes;

  // Persistent robot state
  std::string mRobotDescription;
  std::string mFixedFrame;
  std::string mTfPrefix;
  std::string mParameterNodeName;
  std::string mParameterName;

  // Cached MoveIt objects for IK
  std::unique_ptr<robot_model_loader::RobotModelLoader> RobotModelLoaderPtr;
  std::shared_ptr<moveit::core::RobotModel> RobotModelPtr;
  const moveit::core::JointModelGroup* JointModelGroupPtr = nullptr;
  planning_scene_monitor::PlanningSceneMonitorPtr PlanningSceneMonitorPtr;
  std::string IKGroupName;

  // KDL solvers
  std::unique_ptr<KDL::Chain> KDLChain;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> KDLFkSolver;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> KDLIkSolverVel;
  std::unique_ptr<KDL::ChainIkSolverPos_NR> KDLIkSolver;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> KDLIkSolverJL;
  KDL::JntArray KDLJointMin;
  KDL::JntArray KDLJointMax;
  std::string KDLRootLink;
  std::string KDLTipLink;
  bool KDLUseJointLimits = false;
};

#endif // __vtkMRMLROS2RobotNodeInternals_h
