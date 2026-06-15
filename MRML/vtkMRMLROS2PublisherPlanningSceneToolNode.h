#ifndef __vtkMRMLROS2PublisherPlanningSceneToolNode_h
#define __vtkMRMLROS2PublisherPlanningSceneToolNode_h

#include <vtkMRMLROS2PublisherNode.h>
#include <vtkSlicerROS2ModuleMRMLExport.h>

class vtkMatrix4x4;
class vtkMRMLModelNode;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2PublisherPlanningSceneToolNode:
  public vtkMRMLROS2PublisherNode
{
public:
  typedef vtkMRMLROS2PublisherPlanningSceneToolNode SelfType;
  vtkTypeMacro(vtkMRMLROS2PublisherPlanningSceneToolNode, vtkMRMLROS2PublisherNode);

  static SelfType * New(void);
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  void SetSourceNodeID(const char* sourceNodeID);
  const char* GetSourceNodeID();
  vtkMRMLModelNode* GetSourceNode();

  size_t PublishAttach(vtkMRMLModelNode* modelNode,
                       const char* linkName,
                       const char* touchLinksCsv = nullptr,
                       const char* subframeName = nullptr,
                       vtkMatrix4x4* subframePose = nullptr);

  size_t PublishAttach(const char* linkName,
                       const char* touchLinksCsv = nullptr,
                       const char* subframeName = nullptr,
                       vtkMatrix4x4* subframePose = nullptr);

  size_t PublishDetach(const char* objectId, const char* linkName = nullptr);
  size_t PublishDetach(vtkMRMLModelNode* modelNode, const char* linkName = nullptr);

protected:
  vtkMRMLROS2PublisherPlanningSceneToolNode();
  ~vtkMRMLROS2PublisherPlanningSceneToolNode() override;
};

#endif
