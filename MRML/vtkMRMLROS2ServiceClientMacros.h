#ifndef __vtkMRMLROS2ServiceClientMacros_h
#define __vtkMRMLROS2ServiceClientMacros_h


#define VTK_MRML_ROS_SERVICE_CLIENT_NATIVE_H(slicer_type, name)		\
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
    size_t RequestService(const slicer_type & message);			\
    									\
  protected:								\
    vtkMRMLROS2ServiceClient##name##Node();					\
    ~vtkMRMLROS2ServiceClient##name##Node();				\
  }


#define VTK_MRML_ROS_SERVICE_CLIENT_NATIVE_CXX(slicer_type, ros_type, name) \
  								       \
  vtkStandardNewMacro(vtkMRMLROS2ServiceClient##name##Node);		\
  									\
  typedef vtkMRMLROS2ServiceClientNativeInternals<slicer_type, ros_type >	\
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
  size_t vtkMRMLROS2ServiceClient##name##Node::RequestService(const slicer_type & message) \
  {									\
    mNumberOfCalls++;							\
    const auto justSent = (reinterpret_cast<vtkMRMLROS2ServiceClient##name##Internals *>(mInternals))->RequestService(message); \
    mNumberOfMessagesSent += justSent;					\
    return justSent;							\
  }


#define VTK_MRML_ROS_SERVICE_CLIENT_VTK_H(slicer_type, name)		\
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
    size_t RequestService(slicer_type* msg);				\
    size_t RequestService(vtkSmartPointer<slicer_type> message);	      \
    									\
  protected:								\
    vtkMRMLROS2ServiceClient##name##Node();                                \
    ~vtkMRMLROS2ServiceClient##name##Node();				\
  }


#define VTK_MRML_ROS_SERVICE_CLIENT_VTK_CXX( slicer_type, ros_type, name)	\
  									\
  vtkStandardNewMacro(vtkMRMLROS2ServiceClient##name##Node);		\
  									\
  typedef vtkMRMLROS2ServiceClientVTKInternals<slicer_type, ros_type>	\
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
  size_t vtkMRMLROS2ServiceClient##name##Node::RequestService(slicer_type * message) \
  {									\
    mNumberOfCalls++;							\
    const auto justSent = (reinterpret_cast<vtkMRMLROS2ServiceClient##name##Internals *>(mInternals))->RequestService(message); \
    mNumberOfRequestsSent += justSent;					\
    return justSent;							\
  }									\
									\
  size_t  vtkMRMLROS2ServiceClient##name##Node::RequestService(vtkSmartPointer<slicer_type> message) \
  {									\
    return this->RequestService(message.GetPointer());				\
  }

#endif // __vtkMRMLROS2ServiceClientMacros_h
