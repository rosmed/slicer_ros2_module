#ifndef __vtkMRMLROS2Tf2BufferNode_h
#define __vtkMRMLROS2Tf2BufferNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2Tf2BufferInternals;
class vtkMRMLROS2Tf2BufferLookupNode;
class vtkMRMLTransformNode;
class vtkMatrix4x4;
class vtkObject;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2Tf2BufferNode: public vtkMRMLNode
{

  // friend declarations
  friend class vtkMRMLROS2Tf2BufferInternals;

 public:

  typedef vtkMRMLROS2Tf2BufferNode SelfType;
  vtkTypeMacro(vtkMRMLROS2Tf2BufferNode, vtkMRMLNode);
  static SelfType * New(void);
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;
  void PrintSelf(std::ostream& os, vtkIndent indent) override;
  
  bool AddToROS2Node(const char * nodeId);

  bool AddLookupAndCreateNode(vtkMRMLROS2Tf2BufferLookupNode * lookupNode);

  bool AddLookupForExistingNode(const std::string transformID, vtkMRMLROS2Tf2BufferLookupNode * lookupNode);

  // Save and load
  virtual void ReadXMLAttributes(const char** atts) override;
  virtual void WriteXML(std::ostream& of, int indent) override;

 protected:
  vtkMRMLROS2Tf2BufferNode();
  ~vtkMRMLROS2Tf2BufferNode();

  std::unique_ptr<vtkMRMLROS2Tf2BufferInternals> mInternals;
  std::string mMRMLNodeName = "ros2:tf2buffer";
  size_t mNumberOfBroadcasts = 0;

};

#endif // _vtkMRMLROS2Tf2BufferNode_h
