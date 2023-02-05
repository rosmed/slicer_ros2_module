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

  inline const std::string GetROS2RobotName(void) const {
    return mROS2RobotName;
  }

  void SetRobotName(const std::string & robotName);

  bool AddToROS2Node(const char * nodeId);
  bool SetRobotDescriptionParameterNode(vtkMRMLROS2ParameterNode * param);
  void ObserveParameterNode(vtkMRMLROS2ParameterNode * node);


  bool ParseRobotDescription(void);
  void InitializeLookupListFromURDF(void);
  void InitializeOffsetListAndModelFilesFromURDF(void);

  void InitializeLookups(void);
  void InitializeOffsets(void);
  void LoadLinkModels(void);
  void SetupTransformTree(void);
  void SetupRobotVisualization(void);

  // Save and load
  void ReadXMLAttributes(const char** atts) override;
  void WriteXML(std::ostream& of, int indent) override;

 protected:
  vtkMRMLROS2RobotNode();
  ~vtkMRMLROS2RobotNode();

//   std::unique_ptr<vtkMRMLROS2RobotInternals> mInternals;
//   vtkSmartPointer<vtkMRMLROS2Tf2BufferNode> mBuffer; // enforce a single buffer per node - if using tf on that node we know we need a buffer - if not don't use it
//   std::string mMRMLNodeName = "ros2:node:undefined";
  void ObserveParameterNodeCallback( vtkObject* caller, unsigned long, void* vtkNotUsed(callData));

  std::string mROS2RobotName = "undefined";
  std::string mRobotDescription = "";
  std::string mMRMLNodeName = "ros2:robotnode";
  vtkSmartPointer<vtkMRMLROS2NodeNode> mROS2Node;
  vtkSmartPointer<vtkMRMLROS2ParameterNode> mRobotDescriptionParameterNode;
  std::vector<std::string> mLinkNames;
  std::vector<std::string> mLinkParentNames;
  std::unique_ptr<vtkMRMLROS2RobotNodeInternals> mInternals;
  std::vector<vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> > mLookups;
  std::vector<std::string> mLinkModelFiles;
  size_t mNumberOfLinks = 0;

//   std::vector<vtkMRMLROS2ParameterNode* > mParameterNodes;

  // For ReadXMLAttributes
  inline void SetROS2RobotName(const std::string & name) {
    mROS2RobotName = name;
  }
};

#endif // __vtkMRMLROS2RobotNode_h
