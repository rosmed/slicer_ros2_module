#ifndef __vtkMRMLROS2PublisherNode_h
#define __vtkMRMLROS2PublisherNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2PublisherInternals;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2PublisherNode: public vtkMRMLNode
{

  // friend declarations
  friend class vtkMRMLROS2PublisherInternals;

  template <typename _slicer_type, typename _ros_type>
    friend class vtkMRMLROS2PublisherTemplatedInternals;

 public:
  vtkTypeMacro(vtkMRMLROS2PublisherNode, vtkMRMLNode);

  bool AddToROS2Node(const char * nodeId,
		     const std::string & topic);
  bool RemoveFromROS2Node(const char * nodeId,
					     const std::string & topic);
  bool IsAddedToROS2Node(void) const;

  const std::string & GetTopic(void) const {
    return mTopic;
  }

  const char * GetROSType(void) const;

  const char * GetSlicerType(void) const;

  size_t GetNumberOfCalls(void) const {
    return mNumberOfCalls;
  }

  size_t GetNumberOfMessagesSent(void) const {
    return mNumberOfMessagesSent;
  }

  void PrintSelf(std::ostream& os, vtkIndent indent) override;

  // Save and load
  virtual void ReadXMLAttributes(const char** atts) override;
  virtual void WriteXML(std::ostream& of, int indent) override;
  void UpdateScene(vtkMRMLScene *scene) override;

 protected:
  vtkMRMLROS2PublisherNode() = default;
  ~vtkMRMLROS2PublisherNode() = default;

  vtkMRMLROS2PublisherInternals * mInternals;
  std::string mTopic = "undefined";
  std::string mMRMLNodeName = "ros2:sub:undefined";

  size_t mNumberOfCalls = 0;
  size_t mNumberOfMessagesSent = 0;

  // For ReadXMLAttributes
  inline void SetTopic(const std::string & topic) {
    mTopic = topic;
  }
};

#endif // __vtkMRMLROS2PublisherNode_h
