#ifndef __vtkMRMLROS2PublisherCollisionObjectNode_h
#define __vtkMRMLROS2PublisherCollisionObjectNode_h

#include <vtkMRMLROS2PublisherNode.h>
#include <vtkSlicerROS2ModuleMRMLExport.h>

class vtkMRMLModelNode;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2PublisherCollisionObjectNode:
  public vtkMRMLROS2PublisherNode
{
public:
  typedef vtkMRMLROS2PublisherCollisionObjectNode SelfType;
  vtkTypeMacro(vtkMRMLROS2PublisherCollisionObjectNode, vtkMRMLROS2PublisherNode);

  static SelfType * New(void);
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  // Set the source node to be published as a collision object
  void SetSourceNodeID(const char* sourceNodeID);
  const char* GetSourceNodeID();
  vtkMRMLModelNode* GetSourceNode();

  // Publish the current state of the source node
  size_t Publish();

  // Overload to allow manual publishing of a specific model
  size_t Publish(vtkMRMLModelNode* modelNode);

protected:
  vtkMRMLROS2PublisherCollisionObjectNode();
  ~vtkMRMLROS2PublisherCollisionObjectNode() override;

};

#endif
