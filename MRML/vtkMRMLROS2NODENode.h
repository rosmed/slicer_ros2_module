#ifndef __vtkMRMLROS2NODENode_h
#define __vtkMRMLROS2NODENode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2NodeInternals;
class vtkMRMLROS2SubscriberNode;
class vtkMRMLROS2PublisherNode;
#include <vtkMRMLROS2SubscriberNode.h>
#include <vtkMRMLROS2PublisherNode.h> // forward declaration alone doesn't seem to work here

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2NODENode: public vtkMRMLNode
{

  template <typename _ros_type, typename _slicer_type> friend class vtkMRMLROS2SubscriberTemplatedInternals;
  template <typename _slicer_type, typename _ros_type> friend class vtkMRMLROS2PublisherTemplatedInternals;

 public:
  typedef vtkMRMLROS2NODENode SelfType;
  vtkTypeMacro(vtkMRMLROS2NODENode, vtkMRMLNode);
  static SelfType * New(void);
  void PrintSelf(ostream& os, vtkIndent indent) override;
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  void Create(const std::string & nodeName, bool initialize = false);
  void Spin(void);
  vtkMRMLROS2SubscriberNode* GetSubscriberNodeByTopic(const std::string & topic);
  vtkMRMLROS2PublisherNode* GetPublisherNodeByTopic(const std::string & topic);

  // Save and load
  virtual void ReadXMLAttributes( const char** atts ) override;
  virtual void WriteXML( ostream& of, int indent ) override;

  

 protected:
  vtkMRMLROS2NODENode();
  ~vtkMRMLROS2NODENode();

  std::unique_ptr<vtkMRMLROS2NodeInternals> mInternals;
  std::string mMRMLNodeName = "ros2:node:undefined";
  std::string mROS2NodeName = "undefined";

  vtkGetMacro(mROS2NodeName, std::string);
  vtkSetMacro(mROS2NodeName, std::string);
};

#endif // __vtkMRMLROS2NODENode_h
