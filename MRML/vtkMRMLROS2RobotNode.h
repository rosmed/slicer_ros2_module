#ifndef __vtkMRMLROS2RobotNode_h
#define __vtkMRMLROS2RobotNode_h

#include <memory>
#include <optional>
#include <utility>

// MRML includes
#include <vtkMRMLNode.h>
#include <vtkMatrix4x4.h>
#include <vtkDoubleArray.h>
#include <vtkPointSet.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

class vtkMRMLROS2NodeNode;
class vtkMRMLROS2ParameterNode;
class vtkMRMLROS2Tf2LookupNode;
class vtkMRMLModelNode;
class vtkMRMLROS2RobotNodeInternals;

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

  bool RemoveFromROS2Node(const char * nodeId);

  bool CreateGoalStateRobot(vtkMRMLROS2RobotNode * sourceRobot);
  bool RemoveGoalStateRobot();

  bool SetRobotDescriptionParameterNode();
  void ObserveParameterNode(vtkMRMLROS2ParameterNode * node);

  bool ParseRobotDescription(void);
  void SetupRobotVisualization(void);
  
  // Helper for loading model files with fallback.
  // Outputs one or more mesh parts and a color for each part.
  void LoadModelFile(const std::string& filename,
                     std::vector< vtkSmartPointer<vtkPointSet> >& meshParts,
                     std::vector< std::vector<double> >& meshColors);

  // MoveIt IK methods (commented out for faster build)
  bool setupIKmoveit(const std::string & groupName);
  std::string FindIKmoveit(vtkMatrix4x4* targetPose, const std::string& tipLink,const std::vector<double>& seedJointValues,double timeout = 1.0);

  // KDL Setup and IK methods
  bool SetupKDLIKWithLimits(void);
  std::string FindKDLIK(vtkMatrix4x4* targetPose, const std::vector<double>& seedJointValues);

  // KDL Chain information methods
  std::vector<std::string> GetSegments();
  std::vector<std::string> GetJoints();
  std::vector<std::string> FindRootAndTipLinks() const;

  /** Position limits (rad or m) for each joint in the same order as GetJoints(). */
  std::vector<double> GetJointLowerPositionLimits();
  std::vector<double> GetJointUpperPositionLimits();

  /** Maximum velocity (rad/s or m/s) from URDF for each joint in GetJoints() order.
   *  Returns 0.0 for a joint with no URDF velocity limit. */
  std::vector<double> GetJointVelocityLimits();

  /** Joint types ("revolute", "continuous", "prismatic") in GetJoints() order. */
  std::vector<std::string> GetJointTypes();

  vtkMatrix4x4* ComputeKDLFK(const std::vector<double>& jointValues, vtkMatrix4x4* outTransform, const std::string& linkName = "");
  vtkMatrix4x4* ComputeLocalTransform(const std::vector<double>& jointValues, vtkMatrix4x4* outTransform, const std::string& linkName);

  // Remove all visual nodes (models, transforms, lookups) associated with the robot
  void RemoveRobotVisualization();

  // Save and load
  void ReadXMLAttributes(const char** atts) override;
  void WriteXML(std::ostream& of, int indent) override;
  void UpdateScene(vtkMRMLScene *scene) override;

 protected:
  vtkMRMLROS2RobotNode();
  ~vtkMRMLROS2RobotNode();

  void ObserveParameterNodeCallback( vtkObject* caller, unsigned long, void* vtkNotUsed(callData));

  vtkSmartPointer<vtkMRMLROS2ParameterNode> mRobotDescriptionParameterNode;

  std::string mRobotName = "undefined";
  std::string mMRMLNodeName = "ros2:robot";
  vtkSmartPointer<vtkMRMLROS2NodeNode> mMRMLROS2Node;
  std::unique_ptr<vtkMRMLROS2RobotNodeInternals> mInternals;
  size_t mNumberOfLinks = 0;

};

#endif // __vtkMRMLROS2RobotNode_h
