#ifndef __vtkMRMLROS2RobotNode_h
#define __vtkMRMLROS2RobotNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>
#include <vtkMRMLROS2RobotNodeInternals.h>

class vtkMRMLROS2NodeNode;
class vtkMRMLROS2ParameterNode;
class vtkMRMLROS2Tf2LookupNode;

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

  bool AddToROS2Node(const char * nodeId,
		     const std::string & parameterNodeName,
		     const std::string & parameterName = "robot_description");

  bool SetRobotDescriptionParameterNode();
  void ObserveParameterNode(vtkMRMLROS2ParameterNode * node);

  bool ParseRobotDescription(void);
  void InitializeLookupListFromURDF(void);
  void InitializeOffsetListAndModelFilesFromURDF(void);

  void InitializeLookups(void);
  void InitializeOffsetsAndLinkModels(void);
  void SetupTransformTree(void);
  void SetupRobotVisualization(void);

  // Save and load
  void ReadXMLAttributes(const char** atts) override;
  void WriteXML(std::ostream& of, int indent) override;

 protected:
  vtkMRMLROS2RobotNode();
  ~vtkMRMLROS2RobotNode();

  void ObserveParameterNodeCallback( vtkObject* caller, unsigned long, void* vtkNotUsed(callData));

  std::string mRobotName = "undefined";
  std::string mRobotDescription = "";
  std::string mMRMLNodeName = "ros2:robot";
  vtkSmartPointer<vtkMRMLROS2NodeNode> mROS2Node;
  vtkSmartPointer<vtkMRMLROS2ParameterNode> mRobotDescriptionParameterNode;
  std::vector<std::string> mLinkNames;
  std::vector<std::string> mLinkParentNames;
  std::unique_ptr<vtkMRMLROS2RobotNodeInternals> mInternals;
  std::vector<std::string> mLinkModelFiles;
  size_t mNumberOfLinks = 0;
  std::string mParameterNodeName;
  std::string mParameterName;

  // For ReadXMLAttributes
  inline void SetRobotName(const std::string & name) {
    mRobotName = name;
  }
};

#endif // __vtkMRMLROS2RobotNode_h
