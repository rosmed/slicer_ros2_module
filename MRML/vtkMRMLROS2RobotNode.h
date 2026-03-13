#ifndef __vtkMRMLROS2RobotNode_h
#define __vtkMRMLROS2RobotNode_h

// MRML includes
#include <vtkMRMLNode.h>
#include <vtkMatrix4x4.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>
#include <vtkMRMLROS2RobotNodeInternals.h>

#include <moveit_msgs/msg/robot_trajectory.hpp>

// MoveIt kinematics and planning includes
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>

// KDL includes
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>

class vtkMRMLROS2NodeNode;
class vtkMRMLROS2ParameterNode;
class vtkMRMLROS2Tf2LookupNode;
class vtkMRMLModelNode;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2RobotNode: public vtkMRMLNode
{

 public:
  typedef vtkMRMLROS2RobotNode SelfType;
  vtkTypeMacro(vtkMRMLROS2RobotNode, vtkMRMLNode);
  static SelfType * New(void);
  void PrintSelf(std::ostream& os, vtkIndent indent) override;
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  inline const std::string GetRobotName(void) const {
    return mRobotName;
  }

  // For ReadXMLAttributes
  inline void SetRobotName(const std::string & name) {
    mRobotName = name;
    mMRMLNodeName = "ros2:robot:" + name;
    this->SetName(mMRMLNodeName.c_str()); 
  }

  bool AddToROS2Node(const char * nodeId,
                     const std::string & robotName,
		     const std::string & parameterNodeName,
		     const std::string & parameterName = "robot_description",
		     const std::string & fixedFrame = "",
                     const std::string & tfPrefix = "");

  bool SetRobotDescriptionParameterNode();
  void ObserveParameterNode(vtkMRMLROS2ParameterNode * node);

  bool ParseRobotDescription(void);
  void InitializeLookupListFromURDF(void);
  void InitializeOffsetListAndModelFilesFromURDF(void);

  void InitializeLookups(void);
  void InitializeOffsetsAndLinkModels(void);
  void SetupTransformTree(void);
  void SetupRobotVisualization(void);

  // MoveIt IK methods (commented out for faster build)
  bool setupIKmoveit(const std::string & groupName);
  std::string FindIKmoveit(vtkMatrix4x4* targetPose, 
                     const std::string& tipLink,
                     const std::vector<double>& seedJointValues,
                     double timeout = 1.0);

  // KDL Setup and IK methods
  bool SetupKDLIKWithLimits(void);
  std::string FindKDLIK(vtkMatrix4x4* targetPose,const std::vector<double>& seedJointValues);

  // KDL Chain information methods
  std::vector<std::string> GetSegments();
  std::vector<std::string> GetJoints();
  
  vtkMatrix4x4* ComputeKDLFK(const std::vector<double>& jointValues, vtkMatrix4x4* outTransform,const std::string& linkName = "");
  
  vtkMatrix4x4* ComputeLocalTransform(const std::vector<double>& jointValues, vtkMatrix4x4* outTransform, const std::string& linkName);

  // Plan a joint-space trajectory using MoveIt for the given group.
  // goalJointValues must match the group's joint order. Returns empty trajectory on failure.
  moveit_msgs::msg::RobotTrajectory PlanMoveItTrajectory(const std::string& groupName,
                                                         const std::vector<double>& goalJointValues,
                                                         double velocityScaling = 0.5,
                                                         double accelerationScaling = 0.5,
                                                         double planningTimeSec = 2.0);

  // Python-friendly wrapper: returns JSON string of waypoints
  // Format: {"joint_names": [...], "points": [{"positions": [...], "velocities": [...], "time_from_start": sec}, ...]}
  std::string PlanMoveItTrajectoryJSON(const std::string& groupName,
                                       const std::vector<double>& goalJointValues,
                                       double velocityScaling = 0.5,
                                       double accelerationScaling = 0.5,
                                       double planningTimeSec = 2.0);

  // Execute a previously planned trajectory using MoveIt
  // Returns true on successful execution, false otherwise
  bool ExecuteMoveItTrajectory(const std::string& groupName,
                               const moveit_msgs::msg::RobotTrajectory& trajectory);

  // Execute the cached trajectory from the last PlanMoveItTrajectoryJSON call
  // Returns true on successful execution, false otherwise
  bool ExecuteCachedMoveItTrajectory(const std::string& groupName);

  // Plan and execute a joint-space trajectory in one call
  // Returns true on successful execution, false otherwise
  bool PlanAndExecuteMoveItTrajectory(const std::string& groupName,
                                      const std::vector<double>& goalJointValues,
                                      double velocityScaling = 0.5,
                                      double accelerationScaling = 0.5,
                                      double planningTimeSec = 2.0);

  // Execute trajectory asynchronously (non-blocking) in a background thread
  // Updates the main robot model in real-time during execution
  // Returns immediately, does not block UI
  bool ExecuteMoveItTrajectoryAsync(const std::string& groupName,
                                     const moveit_msgs::msg::RobotTrajectory& trajectory);

  // Save and load
  void ReadXMLAttributes(const char** atts) override;
  void WriteXML(std::ostream& of, int indent) override;
  void UpdateScene(vtkMRMLScene *scene) override;

 protected:
  vtkMRMLROS2RobotNode();
  ~vtkMRMLROS2RobotNode();

  void ObserveParameterNodeCallback( vtkObject* caller, unsigned long, void* vtkNotUsed(callData));

  struct {
    std::vector<std::string> mLinkNames;
    std::vector<std::string> mLinkParentNames;
    std::vector<std::string> mLinkModelFiles;
    std::vector<double> mRobotScale;
    std::vector<vtkSmartPointer<vtkMRMLModelNode>> mLinkModels;
    std::vector<vtkSmartPointer<vtkMRMLROS2Tf2LookupNode>> mLookupNodes;
    std::string mRobotDescription = "";
    vtkSmartPointer<vtkMRMLROS2ParameterNode> mRobotDescriptionParameterNode;
    std::string mParameterNodeName;
    std::string mParameterName;
    std::string mFixedFrame;
    std::string mTfPrefix;
  } mNthRobot;

  std::string mRobotName = "undefined";
  std::string mMRMLNodeName = "ros2:robot";
  vtkSmartPointer<vtkMRMLROS2NodeNode> mMRMLROS2Node;
  std::unique_ptr<vtkMRMLROS2RobotNodeInternals> mInternals;
  size_t mNumberOfLinks = 0;

  // Cached MoveIt objects for IK
  std::unique_ptr<robot_model_loader::RobotModelLoader> RobotModelLoaderPtr;
  moveit::core::RobotModelPtr RobotModelPtr;
  const moveit::core::JointModelGroup* JointModelGroupPtr = nullptr;
  std::string IKGroupName;

  // Cached trajectory for execution after planning
  moveit_msgs::msg::RobotTrajectory CachedTrajectory;

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

#endif // __vtkMRMLROS2RobotNode_h
