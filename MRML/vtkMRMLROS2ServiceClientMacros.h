#ifndef __vtkMRMLROS2ServiceClientMacros_h
#define __vtkMRMLROS2ServiceClientMacros_h


#define VTK_MRML_ROS_SERVICE_CLIENT_NATIVE_H(slicer_type_in, slicer_type_out, name)		\
  class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2ServiceClient##name##Node: \
    public vtkMRMLROS2ServiceClientNode					\
  {									\
  public:								\
    typedef vtkMRMLROS2ServiceClient##name##Node SelfType;			\
    vtkTypeMacro(vtkMRMLROS2ServiceClient##name##Node, vtkMRMLROS2ServiceClientNode); \
									\
    static SelfType * New(void);					\
    vtkMRMLNode * CreateNodeInstance(void) override;			\
    const char * GetNodeTagName(void) override;				\
    size_t SendAsyncRequest(const slicer_type_in & message);			\
    									\
  protected:								\
    vtkMRMLROS2ServiceClient##name##Node();					\
    ~vtkMRMLROS2ServiceClient##name##Node();				\
  }


#define VTK_MRML_ROS_SERVICE_CLIENT_NATIVE_CXX(slicer_type_in, slicer_type_out, ros_type, name) \
  								       \
  vtkStandardNewMacro(vtkMRMLROS2ServiceClient##name##Node);		\
  									\
  typedef vtkMRMLROS2ServiceClientNativeInternals<slicer_type_in, slicer_type_out, ros_type >	\
  vtkMRMLROS2ServiceClient##name##Internals;				\
  									\
  vtkMRMLROS2ServiceClient##name##Node::vtkMRMLROS2ServiceClient##name##Node()	\
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
  									\
  size_t vtkMRMLROS2ServiceClient##name##Node::SendAsyncRequest(const slicer_type_in & message) \
  {									\
    mNumberOfCalls++;							\
    const auto justSent = (reinterpret_cast<vtkMRMLROS2ServiceClient##name##Internals *>(mInternals))->SendAsyncRequest(message); \
    mNumberOfMessagesSent += justSent;					\
    return justSent;							\
  }


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
    size_t SendAsyncRequest(slicer_type_in* msg);				\
    size_t SendAsyncRequest(vtkSmartPointer<slicer_type_in> message);	      \
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
 									\
  size_t vtkMRMLROS2ServiceClient##name##Node::SendAsyncRequest(slicer_type_in * message) \
  {									\
    mNumberOfCalls++;							\
    const auto justSent = (reinterpret_cast<vtkMRMLROS2ServiceClient##name##Internals *>(mInternals))->SendAsyncRequest(message); \
    mNumberOfRequestsSent += justSent;					\
    return justSent;							\
  }									\
									\
  size_t  vtkMRMLROS2ServiceClient##name##Node::SendAsyncRequest(vtkSmartPointer<slicer_type_in> message) \
  {									\
    return this->SendAsyncRequest(message.GetPointer());				\
  }

#endif // __vtkMRMLROS2ServiceClientMacros_h
