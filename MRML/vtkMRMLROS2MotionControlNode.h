#ifndef __vtkMRMLROS2MotionControlNode_h
#define __vtkMRMLROS2MotionControlNode_h

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// Forward declarations
class vtkMoveitMsgsRobotTrajectory;
class vtkMRMLROS2MotionControlNodeInternals;

/**
 * MRML node that owns MoveIt motion-planning and trajectory-execution for a
 * robot.  The robot's vtkMRMLROS2RobotNode provides IK (FindIKmoveit /
 * setupIKmoveit); this node provides the heavier planning workflow so that the
 * two concerns stay separate.
 *
 * Required node references (set before calling planning methods):
 *   "node"  → vtkMRMLROS2NodeNode   (provides the rclcpp::Node)
 *   "robot" → vtkMRMLROS2RobotNode  (informational / future use)
 */
class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2MotionControlNode: public vtkMRMLNode
{
 public:
  typedef vtkMRMLROS2MotionControlNode SelfType;
  vtkTypeMacro(vtkMRMLROS2MotionControlNode, vtkMRMLNode);
  static SelfType * New(void);
  void PrintSelf(std::ostream & os, vtkIndent indent) override;
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  // Convenience setters for the two required node references.
  void SetROS2NodeID(const char * id)   { SetNodeReferenceID("node",  id); }
  void SetRobotNodeID(const char * id)  { SetNodeReferenceID("robot", id); }

  // ── Planning & execution ─────────────────────────────────────────────────

  /** Plan a joint-space trajectory using MoveIt for *groupName*.
   *  goalJointValues must match the group's joint order.
   *  The result is also cached for ExecuteCachedMoveItTrajectory().
   *  Returns a (possibly empty) trajectory; caller owns the returned object. */
  vtkMoveitMsgsRobotTrajectory* PlanMoveItTrajectory(const std::string & groupName,
                                                      const std::vector<double> & goalJointValues,
                                                      double velocityScaling     = 0.5,
                                                      double accelerationScaling = 0.5,
                                                      double planningTimeSec     = 5.0);

  /** Execute a trajectory (blocking). */
  bool ExecuteMoveItTrajectory(const std::string & groupName,
                               vtkMoveitMsgsRobotTrajectory* trajectory);

  /** Execute the trajectory cached by the most recent PlanMoveItTrajectory
   *  call (non-blocking, via detached thread). */
  bool ExecuteCachedMoveItTrajectory(const std::string & groupName);

  /** Plan and execute in one blocking call. */
  bool PlanAndExecuteMoveItTrajectory(const std::string & groupName,
                                      const std::vector<double> & goalJointValues,
                                      double velocityScaling     = 0.5,
                                      double accelerationScaling = 0.5,
                                      double planningTimeSec     = 5.0);

  /** Execute a trajectory asynchronously (non-blocking, detached thread). */
  bool ExecuteMoveItTrajectoryAsync(const std::string & groupName,
                                    vtkMoveitMsgsRobotTrajectory* trajectory);

  // Save / load
  void ReadXMLAttributes(const char** atts) override;
  void WriteXML(std::ostream & of, int indent) override;

 protected:
  vtkMRMLROS2MotionControlNode();
  ~vtkMRMLROS2MotionControlNode();

  /** Return the rclcpp::Node shared pointer from the "node" reference,
   *  or nullptr with an error macro if unavailable. */
  std::shared_ptr<rclcpp::Node> GetROSNodePointer();

  std::unique_ptr<vtkMRMLROS2MotionControlNodeInternals> mInternals;
};

#endif // __vtkMRMLROS2MotionControlNode_h
