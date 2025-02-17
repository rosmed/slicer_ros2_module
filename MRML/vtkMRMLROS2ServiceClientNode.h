#ifndef __vtkMRMLROS2ServiceClientNode_h
#define __vtkMRMLROS2ServiceClientNode_h

// MRML includes
#include <vtkMRMLNode.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>

// forward declaration for internals
class vtkMRMLROS2ServiceClientInternals;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2ServiceClientNode: public vtkMRMLNode
{

  // friend declarations
  friend class vtkMRMLROS2ServiceClientInternals;

  template <typename _slicer_type_in, typename _slicer_type_out, typename _ros_type>
    friend class vtkMRMLROS2ServiceClientTemplatedInternals;

 public:
  vtkTypeMacro(vtkMRMLROS2ServiceClientNode, vtkMRMLNode);

  bool AddToROS2Node(const char * nodeId,
		     const std::string & service);
  bool RemoveFromROS2Node(const char * nodeId,
                          const std::string & service);
  bool IsAddedToROS2Node(void) const;

  const std::string & GetService(void) const {
    return mService;
  }

  const char * GetROSType(void) const;

  const char * GetSlicerTypeIn(void) const;

  const char * GetSlicerTypeOut(void) const;

  size_t GetNumberOfCalls(void) const {
    return mNumberOfCalls;
  }

  size_t GetNumberOfRequestsSent(void) const {
    return mNumberOfRequestsSent;
  }

  void PrintSelf(std::ostream& os, vtkIndent indent) override;

  // Save and load
  virtual void ReadXMLAttributes(const char** atts) override;
  virtual void WriteXML(std::ostream& of, int indent) override;
  void UpdateScene(vtkMRMLScene *scene) override;

 protected:
  vtkMRMLROS2ServiceClientNode() = default;
  ~vtkMRMLROS2ServiceClientNode() = default;

  vtkMRMLROS2ServiceClientInternals * mInternals;
  std::string mService = "undefined";
  std::string mMRMLNodeName = "ros2:srv:client:undefined";

  size_t mNumberOfCalls = 0;
  size_t mNumberOfRequestsSent = 0;

  // For ReadXMLAttributes
  inline void SetService(const std::string & service) {
    mService = service;
  }
};

#endif // __vtkMRMLROS2ServiceClientNode_h
