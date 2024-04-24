#ifndef __vtkMRMLROS2PublisherMacros_h
#define __vtkMRMLROS2PublisherMacros_h


#define VTK_MRML_ROS_PUBLISHER_NATIVE_H(slicer_type, name)		\
  class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2Publisher##name##Node: \
    public vtkMRMLROS2PublisherNode					\
  {									\
  public:								\
    typedef vtkMRMLROS2Publisher##name##Node SelfType;			\
    vtkTypeMacro(vtkMRMLROS2Publisher##name##Node, vtkMRMLROS2PublisherNode); \
									\
    static SelfType * New(void);					\
    vtkMRMLNode * CreateNodeInstance(void) override;			\
    const char * GetNodeTagName(void) override;				\
    size_t Publish(const slicer_type & message);			\
    									\
  protected:								\
    vtkMRMLROS2Publisher##name##Node();					\
    ~vtkMRMLROS2Publisher##name##Node();				\
  }


#define VTK_MRML_ROS_PUBLISHER_NATIVE_CXX(slicer_type, ros_type, name) \
  								       \
  vtkStandardNewMacro(vtkMRMLROS2Publisher##name##Node);		\
  									\
  typedef vtkMRMLROS2PublisherNativeInternals<slicer_type, ros_type >	\
  vtkMRMLROS2Publisher##name##Internals;				\
  									\
  vtkMRMLROS2Publisher##name##Node::vtkMRMLROS2Publisher##name##Node()	\
  {									\
    mInternals = new vtkMRMLROS2Publisher##name##Internals(this);	\
  }									\
  									\
  vtkMRMLROS2Publisher##name##Node::~vtkMRMLROS2Publisher##name##Node() \
  {									\
    delete mInternals;							\
  }									\
  									\
  vtkMRMLNode * vtkMRMLROS2Publisher##name##Node::CreateNodeInstance(void) \
  {									\
    return SelfType::New();						\
  }									\
  									\
  const char * vtkMRMLROS2Publisher##name##Node::GetNodeTagName(void)	\
  {									\
    return "ROS2Publisher"#name;					\
  }									\
  									\
  size_t vtkMRMLROS2Publisher##name##Node::Publish(const slicer_type & message) \
  {									\
    mNumberOfCalls++;							\
    const auto justSent = (reinterpret_cast<vtkMRMLROS2Publisher##name##Internals *>(mInternals))->Publish(message); \
    mNumberOfMessagesSent += justSent;					\
    return justSent;							\
  }


#define VTK_MRML_ROS_PUBLISHER_VTK_H(slicer_type, name)		\
  class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2Publisher##name##Node: \
    public vtkMRMLROS2PublisherNode					\
  {									\
  public:								\
    typedef vtkMRMLROS2Publisher##name##Node SelfType;			\
    vtkTypeMacro(vtkMRMLROS2Publisher##name##Node, vtkMRMLROS2PublisherNode);	\
    									\
    static SelfType * New(void);                                        \
    vtkMRMLNode * CreateNodeInstance(void) override;			\
    const char * GetNodeTagName(void) override;				\
    size_t Publish(vtkSmartPointer<slicer_type> message);	      \
    									\
  protected:								\
    vtkMRMLROS2Publisher##name##Node();                                \
    ~vtkMRMLROS2Publisher##name##Node();				\
  }


#define VTK_MRML_ROS_PUBLISHER_VTK_CXX( slicer_type, ros_type, name)	\
  									\
  vtkStandardNewMacro(vtkMRMLROS2Publisher##name##Node);		\
  									\
  typedef vtkMRMLROS2PublisherVTKInternals<slicer_type, ros_type>	\
  vtkMRMLROS2Publisher##name##Internals;				\
 									\
  vtkMRMLROS2Publisher##name##Node::vtkMRMLROS2Publisher##name##Node() \
  {									\
    mInternals = new vtkMRMLROS2Publisher##name##Internals(this);	\
  }									\
 									\
  vtkMRMLROS2Publisher##name##Node::~vtkMRMLROS2Publisher##name##Node() \
  {									\
    delete mInternals;							\
  }									\
 									\
  vtkMRMLNode * vtkMRMLROS2Publisher##name##Node::CreateNodeInstance(void) \
  {									\
    return SelfType::New();						\
  }									\
 									\
  const char * vtkMRMLROS2Publisher##name##Node::GetNodeTagName(void)	\
  {									\
    return "ROS2Publisher"#name;					\
  }									\
 									\
  size_t  vtkMRMLROS2Publisher##name##Node::Publish(vtkSmartPointer<slicer_type> message) \
  {									\
    mNumberOfCalls++;							\
    const auto justSent = (reinterpret_cast<vtkMRMLROS2Publisher##name##Internals *>(mInternals))->Publish(message.GetPointer()); \
    mNumberOfMessagesSent += justSent;					\
    return justSent;							\
  }

#endif // __vtkMRMLROS2PublisherMacros_h
