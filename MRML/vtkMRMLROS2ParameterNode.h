#ifndef __vtkMRMLROS2ParameterNode_h
#define __vtkMRMLROS2ParameterNode_h

// MRML includes
#include <vtkMRMLNode.h>
#include <utility>
#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2ParameterInternals;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2ParameterNode: public vtkMRMLNode
{

  // friend declarations
  friend class vtkMRMLROS2ParameterInternals;

 public:
  vtkTypeMacro(vtkMRMLROS2ParameterNode, vtkMRMLNode);

  //newly added

  typedef vtkMRMLROS2ParameterNode SelfType;
  typedef std::pair<std::string,std::string> ParameterKey; // pair: {nodeName, parameterName}
  static SelfType * New(void);
  void PrintSelf(ostream& os, vtkIndent indent) override;
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  bool AddToROS2Node(const char * nodeId);

  bool IsAddedToROS2Node(void) const;

  /*! Add a node and parameter to monitor */
  bool AddParameter(std::string nodeName, std::string parameterName);

  std::string GetParameterType(std::string nodeName, std::string parameterName);

  std::string GetParameterValueAsString(std::string nodeName, std::string parameterName);
  std::string GetParameterString(std::string nodeName, std::string parameterName);
  int GetParameterInteger(std::string nodeName, std::string parameterName);

  // Save and load
  // virtual void ReadXMLAttributes(const char** atts) override;
  // virtual void WriteXML(std::ostream& of, int indent) override;
  // void UpdateScene(vtkMRMLScene *scene) override;

  // for debugging only - will be removed
  void listTrackedParameters();

 protected:
  vtkMRMLROS2ParameterNode();
  ~vtkMRMLROS2ParameterNode();

  vtkMRMLROS2ParameterInternals * mInternals = nullptr;
  std::string mMRMLNodeName = "ros2:parameterNode";

  // For ReadXMLAttributes
  // inline void SetTopic(const std::string & trackedNodeName) {
  //   mTopic = trackedNodeName;
  // }



};

#endif // __vtkMRMLROS2ParameterNode_h
