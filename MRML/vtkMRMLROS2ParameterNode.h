#ifndef __vtkMRMLROS2ParameterNode_h
#define __vtkMRMLROS2ParameterNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2ParameterInternals;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2ParameterNode: public vtkMRMLNode
{

  // friend declarations
  friend class vtkMRMLROS2ParameterInternals;

 public:
  vtkTypeMacro(vtkMRMLROS2ParameterNode, vtkMRMLNode);

  bool AddToROS2Node(const char * nodeId,
		     const std::string & trackedNodeName);

  bool IsAddedToROS2Node(void) const;

  const std::string & GetTopic(void) const {
    return mTopic;
  }

  size_t GetNumberOfMessages(void) const {
    return mNumberOfMessages;
  }

  void PrintSelf(ostream& os, vtkIndent indent) override;

  // Save and load
  virtual void ReadXMLAttributes(const char** atts) override;
  virtual void WriteXML(std::ostream& of, int indent) override;
  void UpdateScene(vtkMRMLScene *scene) override;

  //newly added

  typedef vtkMRMLROS2ParameterNode SelfType;
  static SelfType * New(void);
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

 protected:
  vtkMRMLROS2ParameterNode();
  ~vtkMRMLROS2ParameterNode();

  vtkMRMLROS2ParameterInternals * mInternals = nullptr;
  std::string mTopic = "undefined";
  std::string mMRMLNodeName = "ros2:sub:undefined";
  size_t mNumberOfMessages = 0;

  // For ReadXMLAttributes
  inline void SetTopic(const std::string & trackedNodeName) {
    mTopic = trackedNodeName;
  }
};

#endif // __vtkMRMLROS2ParameterNode_h
