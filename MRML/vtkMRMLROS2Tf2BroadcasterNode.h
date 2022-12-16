#ifndef __vtkMRMLROS2Tf2BroadcasterNode_h
#define __vtkMRMLROS2Tf2BroadcasterNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2Tf2BroadcasterInternals;
class vtkMRMLTransformNode;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2Tf2BroadcasterNode: public vtkMRMLNode
{

  // friend declarations
  friend class vtkMRMLROS2Tf2BroadcasterInternals;

 public:
  typedef vtkMRMLROS2Tf2BroadcasterNode SelfType;
  vtkTypeMacro(vtkMRMLROS2Tf2BroadcasterNode, vtkMRMLNode);
  static SelfType * New(void);
  void PrintSelf(std::ostream& os, vtkIndent indent) override;
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  void Create(const std::string & nodeName, bool initialize = false);

  bool AddToROS2Node(const char * nodeId);

  size_t Broadcast(vtkMRMLTransformNode * message, const std::string & parent_id, const std::string & child_id);

  // Save and load
  virtual void ReadXMLAttributes(const char** atts) override;
  virtual void WriteXML(std::ostream& of, int indent) override;

 protected:
  vtkMRMLROS2Tf2BroadcasterNode();
  ~vtkMRMLROS2Tf2BroadcasterNode();
  
  std::unique_ptr<vtkMRMLROS2Tf2BroadcasterInternals> mInternals;
  std::string mMRMLNodeName = "ros2:tf2:broadcaster";

};

#endif // _vtkMRMLROS2Tf2BroadcasterNode_h
