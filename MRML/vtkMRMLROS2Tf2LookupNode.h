#ifndef __vtkMRMLROS2Tf2LookupNode_h
#define __vtkMRMLROS2Tf2LookupNode_h

// MRML includes
#include <vtkMRMLTransformNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLNode;
class vtkMatrix4x4;
class vtkObject;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2Tf2LookupNode: public vtkMRMLTransformNode 
{
 public:

  typedef vtkMRMLROS2Tf2LookupNode SelfType;
  vtkTypeMacro(vtkMRMLROS2Tf2LookupNode, vtkMRMLTransformNode);
  static SelfType * New(void);
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;
  void PrintSelf(std::ostream& os, vtkIndent indent) override;

  bool SetParentID(const std::string & parent_id);
  const std::string& GetParentID(void) const;

  bool SetChildID(const std::string & child_id);
  const std::string& GetChildID(void) const;

  bool isParentAndChildSet(void);

  // Save and load
  virtual void ReadXMLAttributes(const char** atts) override;
  virtual void WriteXML(std::ostream& of, int indent) override;

 protected:
  vtkMRMLROS2Tf2LookupNode();
  ~vtkMRMLROS2Tf2LookupNode();
  
  void UpdateMRMLNodeName();

  std::string mMRMLNodeName = "ros2:tf2lookup:empty";
  std::string mParentID = "";
  std::string mChildID = "";
  size_t mNumberOfBroadcasts = 0;

};

#endif // _vtkMRMLROS2Tf2LookupNode_h
