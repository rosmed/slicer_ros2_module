#ifndef __vtkMRMLROS2NodeNode_h
#define __vtkMRMLROS2NodeNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerRos2ModuleLogicExport.h>

// forward declaration for internals
class vtkMRMLROS2NodeInternals;
class vtkMRMLROS2SubscriberNode;

class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkMRMLROS2NodeNode: public vtkMRMLNode
{

  template <typename _ros_type, typename _slicer_type>
    friend class vtkMRMLROS2SubscriberTemplatedInternals;

 public:
  typedef vtkMRMLROS2NodeNode SelfType;
  vtkTypeMacro(vtkMRMLROS2NodeNode, vtkMRMLNode);
  static SelfType * New(void);
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  void Create(const std::string & nodeName, bool initialize = false);
  void Spin(void);
  std::vector<vtkSmartPointer<vtkMRMLROS2SubscriberNode>> mSubs;
  // void SetAndObserveSubscriberNode(const char* nodeId);

 protected:
  vtkMRMLROS2NodeNode();
  ~vtkMRMLROS2NodeNode();

  std::unique_ptr<vtkMRMLROS2NodeInternals> mInternals;
  std::string mMRMLNodeName = "ros2:node:undefined";
};

#endif // __vtkMRMLROS2NodeNode_h
