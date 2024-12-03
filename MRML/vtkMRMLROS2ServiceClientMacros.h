#ifndef __vtkMRMLROS2ServiceClientMacros_h
#define __vtkMRMLROS2ServiceClientMacros_h


#define VTK_MRML_ROS_SERVICE_CLIENT_VTK_H(slicer_type_in, slicer_type_out, name)		\
  class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2ServiceClient##name##Node: \
    public vtkMRMLROS2ServiceClientNode					\
  {									\
  public:								\
    typedef vtkMRMLROS2ServiceClient##name##Node SelfType;			\
    vtkTypeMacro(vtkMRMLROS2ServiceClient##name##Node, vtkMRMLROS2ServiceClientNode);	\
    									\
    static SelfType * New(void);                                        \
    vtkMRMLNode * CreateNodeInstance(void) override;			\
    const char * GetNodeTagName(void) override;				\
    slicer_type_in * CreateBlankRequest(void);				\
    bool PreRequestCheck(void);                      \
    size_t SendAsyncRequest(vtkSmartPointer<slicer_type_in> message);	      \
    slicer_type_out * GetLastResponse(void);				\
    bool GetLastResponseStatus(void);					\
    									\
  protected:								\
    vtkMRMLROS2ServiceClient##name##Node();                                \
    ~vtkMRMLROS2ServiceClient##name##Node();				\
  }


#define VTK_MRML_ROS_SERVICE_CLIENT_VTK_CXX( slicer_type_in, slicer_type_out, ros_type, name)	\
  									\
  vtkStandardNewMacro(vtkMRMLROS2ServiceClient##name##Node);		\
  									\
  typedef vtkMRMLROS2ServiceClientVTKInternals<slicer_type_in, slicer_type_out, ros_type>	\
  vtkMRMLROS2ServiceClient##name##Internals;				\
 									\
  vtkMRMLROS2ServiceClient##name##Node::vtkMRMLROS2ServiceClient##name##Node() \
  {									\
    mInternals = new vtkMRMLROS2ServiceClient##name##Internals(this);	\
  }									\
 									\
  vtkMRMLROS2ServiceClient##name##Node::~vtkMRMLROS2ServiceClient##name##Node() \
  {									\
    delete mInternals;							\
  }									\
 									\
  vtkMRMLNode * vtkMRMLROS2ServiceClient##name##Node::CreateNodeInstance(void) \
  {									\
    return SelfType::New();						\
  }									\
 									\
  const char * vtkMRMLROS2ServiceClient##name##Node::GetNodeTagName(void)	\
  {									\
    return "ROS2ServiceClient"#name;					\
  }									\
  slicer_type_in * vtkMRMLROS2ServiceClient##name##Node::CreateBlankRequest(void) \
  {									\
    vtkSmartPointer<slicer_type_in> result = slicer_type_in::New();	\
    return result.GetPointer();						\
  }     \
            \
  bool vtkMRMLROS2ServiceClient##name##Node::PreRequestCheck(void) \
  {									\
    return (reinterpret_cast<vtkMRMLROS2ServiceClient##name##Internals *>(mInternals))->PreRequestCheck(); \
  }     \
 									\
  size_t vtkMRMLROS2ServiceClient##name##Node::SendAsyncRequest(vtkSmartPointer<slicer_type_in> message) \
  {									\
    mNumberOfCalls++;							\
    const auto justSent = (reinterpret_cast<vtkMRMLROS2ServiceClient##name##Internals *>(mInternals))->SendAsyncRequest(message); \
    mNumberOfRequestsSent += justSent;					\
    return justSent;							\
  }                                                          \
                  \
  slicer_type_out * vtkMRMLROS2ServiceClient##name##Node::GetLastResponse(void) \
  {									\
    slicer_type_out * result; \
    result = (reinterpret_cast<vtkMRMLROS2ServiceClient##name##Internals *>(mInternals))->GetLastResponse(); \
    return result;							\
  }						\
   									\
  bool vtkMRMLROS2ServiceClient##name##Node::GetLastResponseStatus(void) \
  {									\
    return (reinterpret_cast<vtkMRMLROS2ServiceClient##name##Internals *>(mInternals))->GetLastResponseStatus(); \
  }						
    	
#endif // __vtkMRMLROS2ServiceClientMacros_h
