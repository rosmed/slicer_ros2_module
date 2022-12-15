#ifndef __vtkMRMLROS2Tf2BroadcasterNode_h
#define __vtkMRMLROS2Tf2BroadcasterNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// ROS includes 
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

// forward declaration for internals
class vtkMRMLROS2Tf2BroadcasterInternals;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2Tf2BroadcasterNode: public vtkMRMLNode
{

 public:
  typedef vtkMRMLROS2Tf2BroadcasterNode SelfType;
  vtkTypeMacro(vtkMRMLROS2Tf2BroadcasterNode, vtkMRMLNode);
  static SelfType * New(void);
  void PrintSelf(std::ostream& os, vtkIndent indent) override;
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  void Create(const std::string & nodeName, bool initialize = false);

  bool AddToROS2Node(const char * nodeId);

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
