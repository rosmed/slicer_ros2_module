#ifndef __vtkMRMLROS2Tf2BufferNode_h
#define __vtkMRMLROS2Tf2BufferNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2Tf2BufferInternals;
class vtkMRMLROS2Tf2LookupNode;
class vtkMRMLROS2NODENode;
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
  inline const std::string GetBufferNodeName(void) const {
    return mBufferNodeName;
  }
  
  bool AddToROS2Node(const char * nodeId);

  bool AddLookupNode(vtkMRMLROS2Tf2LookupNode * lookupNode);
  vtkMRMLROS2Tf2LookupNode * CreateAndAddLookupNode(const std::string & parent_id, const std::string & child_id);

  bool Spin(void); 
  bool LookupTryCatch(const std::string & parent_id, const std::string & child_id, vtkMRMLROS2Tf2LookupNode * lookupNode);

  // Save and load
  virtual void ReadXMLAttributes(const char** atts) override;
  virtual void WriteXML(std::ostream& of, int indent) override;

  std::vector<vtkSmartPointer<vtkMRMLROS2Tf2LookupNode>> mLookupNodes;

 protected:
  vtkMRMLROS2Tf2BufferNode();
  ~vtkMRMLROS2Tf2BufferNode();

  std::unique_ptr<vtkMRMLROS2Tf2BufferInternals> mInternals;
  vtkSmartPointer<vtkMRMLROS2Tf2BufferNode> mBufferNode;
  std::string mMRMLNodeName = "ros2:tf2buffer";
  size_t mNumberOfBroadcasts = 0;
  std::string mBufferNodeName = "undefined";

  // For ReadXMLAttributes
  inline void SetBufferNodeName(const std::string & name) {
    mBufferNodeName = name;
  }

};

#endif // _vtkMRMLROS2Tf2BufferNode_h
