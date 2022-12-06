#ifndef __vtkMRMLROS2NODENode_h
#define __vtkMRMLROS2NODENode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2NodeInternals;
class vtkMRMLROS2SubscriberNode;
class vtkMRMLROS2PublisherNode;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2NODENode: public vtkMRMLNode
{

  template <typename _ros_type, typename _slicer_type> friend class vtkMRMLROS2SubscriberTemplatedInternals;
  template <typename _slicer_type, typename _ros_type> friend class vtkMRMLROS2PublisherTemplatedInternals;

 public:
  typedef vtkMRMLROS2NODENode SelfType;
  vtkTypeMacro(vtkMRMLROS2NODENode, vtkMRMLNode);
  static SelfType * New(void);
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  void Create(const std::string & nodeName, bool initialize = false);
  void Spin(void);
  vtkMRMLNode* GetSubscriberNodeByTopic(const std::string & topic);

 protected:
  vtkMRMLROS2NODENode();
  ~vtkMRMLROS2NODENode();

  std::unique_ptr<vtkMRMLROS2NodeInternals> mInternals;
  std::string mMRMLNodeName = "ros2:node:undefined";
};

#endif // __vtkMRMLROS2NODENode_h
