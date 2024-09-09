#ifndef __vtkMRMLROS2Tf2BroadcasterNode_h
#define __vtkMRMLROS2Tf2BroadcasterNode_h

#include <memory>

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2Tf2BroadcasterInternals;
class vtkMRMLTransformNode;
class vtkMatrix4x4;
class vtkObject;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2Tf2BroadcasterNode: public vtkMRMLNode
{
  // friend declarations
  friend class vtkMRMLROS2Tf2BroadcasterInternals;

 public:

  typedef vtkMRMLROS2Tf2BroadcasterNode SelfType;
  vtkTypeMacro(vtkMRMLROS2Tf2BroadcasterNode, vtkMRMLNode);
  static SelfType * New(void);
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;
  void PrintSelf(std::ostream& os, vtkIndent indent) override;

  bool AddToROS2Node(const char * nodeId);
  bool RemoveFromROS2Node(const char * nodeId);
  bool IsAddedToROS2Node(void) const;

  bool SetParentID(const std::string & parent_id);
  const std::string & GetParentID(void) const;

  bool SetChildID(const std::string & child_id);
  const std::string & GetChildID(void) const;

  bool IsParentAndChildSet(void);

  bool Broadcast(vtkMRMLTransformNode * message);
  // overloaded to support a transform or a matrix
  bool Broadcast(vtkMatrix4x4 * message);

  void ObserveTransformNode(vtkMRMLTransformNode* node);

  // Save and load
  virtual void ReadXMLAttributes(const char** atts) override;
  virtual void WriteXML(std::ostream& of, int indent) override;
  void UpdateScene(vtkMRMLScene *scene) override;

 protected:
  vtkMRMLROS2Tf2BroadcasterNode();
  ~vtkMRMLROS2Tf2BroadcasterNode();

  void ObserveTransformCallback( vtkObject* caller, unsigned long event, void* callData );

  void UpdateMRMLNodeName();

  std::unique_ptr<vtkMRMLROS2Tf2BroadcasterInternals> mInternals;
  std::string mMRMLNodeName = "ros2:tf2broadcaster:empty";
  std::string mParentID = "";
  std::string mChildID = "";
  size_t mNumberOfBroadcasts = 0;

};

#endif // _vtkMRMLROS2Tf2BroadcasterNode_h
