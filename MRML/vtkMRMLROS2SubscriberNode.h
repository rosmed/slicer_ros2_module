#ifndef __vtkMRMLROS2SubscriberNode_h
#define __vtkMRMLROS2SubscriberNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2SubscriberInternals;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2SubscriberNode: public vtkMRMLNode
{

  // friend declarations
  friend class vtkMRMLROS2SubscriberInternals;

  template <typename _ros_type, typename _slicer_type>
    friend class vtkMRMLROS2SubscriberTemplatedInternals;

 public:
  vtkTypeMacro(vtkMRMLROS2SubscriberNode, vtkMRMLNode);

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

  size_t GetNumberOfMessages(void) const {
    return mNumberOfMessages;
  }

  void PrintSelf(ostream& os, vtkIndent indent) override;

  /**
   * Get the latest ROS message in YAML format
   */
  std::string GetLastMessageYAML(void) const;

  /**
   * Get the latest message as a vtkVariant.  This method will use the
   * latest ROS message received and convert it to the internal
   * Slicer/VTK type if needed.  The result of the conversion is
   * cached so future calls to GetLastMessage don't require converting
   * again
   */
  virtual vtkVariant GetLastMessageVariant(void) = 0;

  // Save and load
  virtual void ReadXMLAttributes(const char** atts) override;
  virtual void WriteXML(std::ostream& of, int indent) override;
  void UpdateScene(vtkMRMLScene *scene) override;

 protected:
  vtkMRMLROS2SubscriberNode() = default;
  ~vtkMRMLROS2SubscriberNode() = default;

  vtkMRMLROS2SubscriberInternals * mInternals = nullptr;
  std::string mTopic = "undefined";
  std::string mMRMLNodeName = "ros2:sub:undefined";
  size_t mNumberOfMessages = 0;

  // For ReadXMLAttributes
  inline void SetTopic(const std::string & topic) {
    mTopic = topic;
  }
};

#endif // __vtkMRMLROS2SubscriberNode_h
