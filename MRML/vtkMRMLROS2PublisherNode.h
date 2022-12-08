#ifndef __vtkMRMLROS2PublisherNode_h
#define __vtkMRMLROS2PublisherNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2PublisherInternals;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2PublisherNode: public vtkMRMLNode
{

  friend class vtkMRMLROS2PublisherInternals;

  template <typename _slicer_type, typename _ros_type>
    friend class vtkMRMLROS2PublisherTemplatedInternals;

 public:
  vtkTypeMacro(vtkMRMLROS2PublisherNode, vtkMRMLNode);

  bool AddToROS2Node(const char * nodeId,
		     const std::string & topic);

  const char * GetTopic(void) const;

  const char * GetROSType(void) const;

  const char * GetSlicerType(void) const;

  size_t GetNumberOfMessages(void) const;

  size_t mNumberOfMessages = 0;

  void PrintSelf(ostream& os, vtkIndent indent) override;

  /**
   * Get the latest ROS message in YAML format
   */
  std::string GetLastMessageYAML(void) const;
  //
  // /**
  //  * Get the latest message as a vtkVariant.  This method will use the
  //  * latest ROS message received and convert it to the internal
  //  * Slicer/VTK type if needed.  The result of the conversion is
  //  * cached so future calls to GetLastMessage don't require converting
  //  * again
  //  */
  // virtual vtkVariant GetLastMessageVariant(void) = 0;

    // Save and load
  virtual void ReadXMLAttributes( const char** atts ) override;
  virtual void WriteXML( ostream& of, int indent ) override;
  void UpdateScene(vtkMRMLScene *scene) override;

 protected:
  vtkMRMLROS2PublisherNode();
  ~vtkMRMLROS2PublisherNode();

  vtkMRMLROS2PublisherInternals * mInternals;
  std::string mTopic = "undefined";
  std::string mMRMLNodeName = "ros2:sub:undefined";
  std::string parentNodeID = "undefined";

  vtkGetMacro(mTopic, std::string);
  vtkSetMacro(mTopic, std::string);
  vtkGetMacro(parentNodeID, std::string);
  vtkSetMacro(parentNodeID, std::string);
};

#endif // __vtkMRMLROS2PublisherNode_h
