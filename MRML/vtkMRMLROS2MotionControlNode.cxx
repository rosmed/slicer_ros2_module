#include <vtkMRMLROS2MotionControlNode.h>

#include <vtkMRMLROS2NodeNode.h>
#include <vtkMRMLROS2NodeInternals.h>

#include <vtkMoveitMsgsRobotTrajectory.h>
#include <vtkROS2ToSlicer.h>
#include <vtkSlicerToROS2.h>

#include <vtkObjectFactory.h>
#include <vtkSmartPointer.h>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <algorithm>
#include <sstream>
#include <thread>

// ── Internals ────────────────────────────────────────────────────────────────

struct vtkMRMLROS2MotionControlNodeInternals
{
  moveit_msgs::msg::RobotTrajectory CachedTrajectory;
};

// ── vtkStandardNewMacro ──────────────────────────────────────────────────────

vtkStandardNewMacro(vtkMRMLROS2MotionControlNode);

// ── Constructor / destructor ─────────────────────────────────────────────────

vtkMRMLROS2MotionControlNode::vtkMRMLROS2MotionControlNode()
  : mInternals(std::make_unique<vtkMRMLROS2MotionControlNodeInternals>())
{}

vtkMRMLROS2MotionControlNode::~vtkMRMLROS2MotionControlNode() = default;

// ── vtkMRMLNode boilerplate ──────────────────────────────────────────────────

vtkMRMLNode * vtkMRMLROS2MotionControlNode::CreateNodeInstance(void)
{
  return SelfType::New();
}

const char * vtkMRMLROS2MotionControlNode::GetNodeTagName(void)
{
  return "ROS2MotionControl";
}

void vtkMRMLROS2MotionControlNode::PrintSelf(std::ostream & os, vtkIndent indent)
{
  Superclass::PrintSelf(os, indent);
}

void vtkMRMLROS2MotionControlNode::ReadXMLAttributes(const char** atts)
{
  Superclass::ReadXMLAttributes(atts);
}

void vtkMRMLROS2MotionControlNode::WriteXML(std::ostream & of, int indent)
{
  Superclass::WriteXML(of, indent);
}

// ── Private helper ───────────────────────────────────────────────────────────

std::shared_ptr<rclcpp::Node> vtkMRMLROS2MotionControlNode::GetROSNodePointer()
{
  auto * nodeRef = vtkMRMLROS2NodeNode::SafeDownCast(GetNodeReference("node"));
  if (!nodeRef || !nodeRef->mInternals || !nodeRef->mInternals->mNodePointer) {
    vtkErrorMacro(<< "GetROSNodePointer: \"node\" reference is not set or not initialized");
    return nullptr;
  }
  return nodeRef->mInternals->mNodePointer;
}

// ── Planning / execution ─────────────────────────────────────────────────────

vtkMoveitMsgsRobotTrajectory* vtkMRMLROS2MotionControlNode::PlanMoveItTrajectory(
    const std::string & groupName,
    const std::vector<double> & goalJointValues,
    double velocityScaling,
    double accelerationScaling,
    double planningTimeSec)
{
  vtkMoveitMsgsRobotTrajectory* traj = vtkMoveitMsgsRobotTrajectory::New();

  auto node = GetROSNodePointer();
  if (!node) { return traj; }

  if (groupName.empty()) {
    vtkErrorMacro(<< "PlanMoveItTrajectory: groupName is empty");
    return traj;
  }

  moveit::planning_interface::MoveGroupInterface moveGroup(node, groupName);

  const auto jointNames = moveGroup.getJointNames();
  if (jointNames.size() != goalJointValues.size()) {
    vtkErrorMacro(<< "PlanMoveItTrajectory: expected " << jointNames.size()
                  << " joint values for group '" << groupName
                  << "' but got " << goalJointValues.size());
    return traj;
  }

  const double velScale = std::clamp(velocityScaling,     0.0, 1.0);
  const double accScale = std::clamp(accelerationScaling, 0.0, 1.0);
  moveGroup.setMaxVelocityScalingFactor(velScale);
  moveGroup.setMaxAccelerationScalingFactor(accScale);
  moveGroup.setPlanningTime(planningTimeSec > 0.0 ? planningTimeSec : 5.0);
  moveGroup.setStartStateToCurrentState();

  std::map<std::string, double> targets;
  for (size_t i = 0; i < jointNames.size(); ++i) {
    targets[jointNames[i]] = goalJointValues[i];
  }
  moveGroup.setJointValueTarget(targets);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = moveGroup.plan(plan);
  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    // Cache the ROS trajectory for ExecuteCachedMoveItTrajectory
    mInternals->CachedTrajectory = plan.trajectory;
    vtkROS2ToSlicer(plan.trajectory, vtkSmartPointer<vtkMoveitMsgsRobotTrajectory>(traj));
  } else {
    vtkErrorMacro(<< "PlanMoveItTrajectory: planning failed for group '" << groupName
                  << "' with MoveItErrorCode=" << result.val);
  }

  return traj;
}

bool vtkMRMLROS2MotionControlNode::ExecuteMoveItTrajectory(
    const std::string & groupName,
    vtkMoveitMsgsRobotTrajectory* trajectory)
{
  auto node = GetROSNodePointer();
  if (!node) { return false; }

  if (groupName.empty()) {
    vtkErrorMacro(<< "ExecuteMoveItTrajectory: groupName is empty");
    return false;
  }
  if (!trajectory) {
    vtkErrorMacro(<< "ExecuteMoveItTrajectory: trajectory is null");
    return false;
  }

  try {
    moveit_msgs::msg::RobotTrajectory ros_traj;
    vtkSlicerToROS2(trajectory, ros_traj, node);

    if (ros_traj.joint_trajectory.points.empty()) {
      vtkErrorMacro(<< "ExecuteMoveItTrajectory: trajectory is empty");
      return false;
    }

    moveit::planning_interface::MoveGroupInterface moveGroup(node, groupName);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory = ros_traj;

    auto result = moveGroup.execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      vtkInfoMacro(<< "ExecuteMoveItTrajectory: success for group '" << groupName << "'");
      return true;
    }
    vtkErrorMacro(<< "ExecuteMoveItTrajectory: failed for group '" << groupName
                  << "' with MoveItErrorCode=" << result.val);
    return false;
  }
  catch (const std::exception & e) {
    vtkErrorMacro(<< "ExecuteMoveItTrajectory: exception - " << e.what());
    return false;
  }
}

bool vtkMRMLROS2MotionControlNode::ExecuteCachedMoveItTrajectory(const std::string & groupName)
{
  if (mInternals->CachedTrajectory.joint_trajectory.points.empty()) {
    vtkErrorMacro(<< "ExecuteCachedMoveItTrajectory: no cached trajectory. "
                     "Call PlanMoveItTrajectory first.");
    return false;
  }

  vtkSmartPointer<vtkMoveitMsgsRobotTrajectory> vtk_traj =
    vtkSmartPointer<vtkMoveitMsgsRobotTrajectory>::New();
  vtkROS2ToSlicer(mInternals->CachedTrajectory, vtk_traj);
  return ExecuteMoveItTrajectoryAsync(groupName, vtk_traj.Get());
}

bool vtkMRMLROS2MotionControlNode::PlanAndExecuteMoveItTrajectory(
    const std::string & groupName,
    const std::vector<double> & goalJointValues,
    double velocityScaling,
    double accelerationScaling,
    double planningTimeSec)
{
  auto * trajectory = PlanMoveItTrajectory(groupName, goalJointValues,
                                            velocityScaling, accelerationScaling,
                                            planningTimeSec);
  auto node = GetROSNodePointer();
  if (!node) { trajectory->Delete(); return false; }

  moveit_msgs::msg::RobotTrajectory ros_traj;
  vtkSlicerToROS2(trajectory, ros_traj, node);

  if (ros_traj.joint_trajectory.points.empty()) {
    vtkErrorMacro(<< "PlanAndExecuteMoveItTrajectory: planning failed, cannot execute");
    trajectory->Delete();
    return false;
  }

  bool result = ExecuteMoveItTrajectory(groupName, trajectory);
  trajectory->Delete();
  return result;
}

bool vtkMRMLROS2MotionControlNode::ExecuteMoveItTrajectoryAsync(
    const std::string & groupName,
    vtkMoveitMsgsRobotTrajectory* trajectory)
{
  auto node = GetROSNodePointer();
  if (!node) { return false; }

  if (!trajectory) {
    vtkErrorMacro(<< "ExecuteMoveItTrajectoryAsync: trajectory is null");
    return false;
  }

  moveit_msgs::msg::RobotTrajectory ros_traj;
  vtkSlicerToROS2(trajectory, ros_traj, node);

  if (ros_traj.joint_trajectory.points.empty()) {
    vtkErrorMacro(<< "ExecuteMoveItTrajectoryAsync: trajectory is empty");
    return false;
  }

  std::thread([node, groupName, ros_traj]() {
    try {
      moveit::planning_interface::MoveGroupInterface moveGroup(node, groupName);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory = ros_traj;
      moveGroup.execute(plan);
    }
    catch (const std::exception & e) {
      // Cannot call vtkErrorMacro from a detached thread safely
      (void)e;
    }
  }).detach();

  return true;
}
