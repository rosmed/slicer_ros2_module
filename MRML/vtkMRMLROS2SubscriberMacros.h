#ifndef __vtkMRMLROS2SubscriberMacros_h
#define __vtkMRMLROS2SubscriberMacros_h

#define VTK_MRML_ROS_SUBSCRIBER_NATIVE_H(slicer_type, name)             \
  class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2Subscriber##name##Node: \
  public vtkMRMLROS2SubscriberNode                                      \
  {                                                                     \
  public:                                                               \
    typedef vtkMRMLROS2Subscriber##name##Node SelfType;                 \
    vtkTypeMacro(vtkMRMLROS2Subscriber##name##Node, vtkMRMLROS2SubscriberNode); \
                                                                        \
    static SelfType * New(void);                                        \
    vtkMRMLNode * CreateNodeInstance(void) override;                    \
    const char * GetNodeTagName(void) override;                         \
    void GetLastMessage(slicer_type & message) const;                   \
    slicer_type GetLastMessage(void) const;                             \
    vtkVariant GetLastMessageVariant(void) override;                    \
                                                                        \
  protected:                                                            \
    vtkMRMLROS2Subscriber##name##Node();                                \
    ~vtkMRMLROS2Subscriber##name##Node();                               \
  }


#define VTK_MRML_ROS_SUBSCRIBER_NATIVE_CXX(ros_type, slicer_type, name) \
                                                                        \
  vtkStandardNewMacro(vtkMRMLROS2Subscriber##name##Node);               \
                                                                        \
  typedef vtkMRMLROS2SubscriberNativeInternals<ros_type, slicer_type>   \
    vtkMRMLROS2Subscriber##name##Internals;                             \
                                                                        \
  vtkMRMLROS2Subscriber##name##Node::vtkMRMLROS2Subscriber##name##Node() \
  {                                                                     \
    mInternals = new vtkMRMLROS2Subscriber##name##Internals(this);      \
  }                                                                     \
                                                                        \
  vtkMRMLROS2Subscriber##name##Node::~vtkMRMLROS2Subscriber##name##Node() \
  {                                                                     \
    delete mInternals;                                                  \
  }                                                                     \
                                                                        \
  vtkMRMLNode * vtkMRMLROS2Subscriber##name##Node::CreateNodeInstance(void) \
  {                                                                     \
    return SelfType::New();                                             \
  }                                                                     \
                                                                        \
  const char * vtkMRMLROS2Subscriber##name##Node::GetNodeTagName(void)  \
  {                                                                     \
    return "ROS2Subscriber"#name;                                       \
  }                                                                     \
                                                                        \
  void vtkMRMLROS2Subscriber##name##Node::GetLastMessage(slicer_type & message) const \
  {                                                                     \
    (reinterpret_cast<vtkMRMLROS2Subscriber##name##Internals *>(mInternals))->GetLastMessage(message); \
  }                                                                     \
                                                                        \
  slicer_type vtkMRMLROS2Subscriber##name##Node::GetLastMessage(void) const \
  {                                                                     \
    slicer_type result;                                                 \
    this->GetLastMessage(result);                                       \
    return result;                                                      \
}                                                                       \
                                                                        \
  vtkVariant vtkMRMLROS2Subscriber##name##Node::GetLastMessageVariant(void) \
  {                                                                     \
    return (reinterpret_cast<vtkMRMLROS2Subscriber##name##Internals *>(mInternals))->GetLastMessageVariant(); \
  }


#define VTK_MRML_ROS_SUBSCRIBER_VTK_H(slicer_type, name)                \
  class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2Subscriber##name##Node: \
  public vtkMRMLROS2SubscriberNode                                      \
  {                                                                     \
  public:                                                               \
    typedef vtkMRMLROS2Subscriber##name##Node SelfType;                 \
    vtkTypeMacro(vtkMRMLROS2Subscriber##name##Node, vtkMRMLROS2SubscriberNode); \
                                                                        \
    static SelfType * New(void);                                        \
    vtkMRMLNode * CreateNodeInstance(void) override;                    \
    const char * GetNodeTagName(void) override;                         \
    void GetLastMessage(slicer_type * message) const;                   \
    slicer_type * GetLastMessage(void) const;                           \
    void GetLastMessage(vtkSmartPointer<slicer_type> message) const;    \
    vtkVariant GetLastMessageVariant(void) override;                    \
                                                                        \
  protected:                                                            \
    vtkMRMLROS2Subscriber##name##Node();                                \
    ~vtkMRMLROS2Subscriber##name##Node();                               \
  }


#define VTK_MRML_ROS_SUBSCRIBER_VTK_CXX(ros_type, slicer_type, name)    \
                                                                        \
  vtkStandardNewMacro(vtkMRMLROS2Subscriber##name##Node);               \
                                                                        \
  typedef vtkMRMLROS2SubscriberVTKInternals<ros_type, slicer_type>      \
    vtkMRMLROS2Subscriber##name##Internals;                             \
                                                                        \
  vtkMRMLROS2Subscriber##name##Node::vtkMRMLROS2Subscriber##name##Node() \
  {                                                                     \
    mInternals = new vtkMRMLROS2Subscriber##name##Internals(this);      \
  }                                                                     \
                                                                        \
  vtkMRMLROS2Subscriber##name##Node::~vtkMRMLROS2Subscriber##name##Node() \
  {                                                                     \
    delete mInternals;                                                  \
  }                                                                     \
                                                                        \
  vtkMRMLNode * vtkMRMLROS2Subscriber##name##Node::CreateNodeInstance(void) \
  {                                                                     \
    return SelfType::New();                                             \
  }                                                                     \
                                                                        \
  const char * vtkMRMLROS2Subscriber##name##Node::GetNodeTagName(void)  \
  {                                                                     \
    return "ROS2Subscriber"#name;                                       \
  }                                                                     \
                                                                        \
  void vtkMRMLROS2Subscriber##name##Node::GetLastMessage(slicer_type * message) const \
  {                                                                     \
    (reinterpret_cast<vtkMRMLROS2Subscriber##name##Internals *>(mInternals))->GetLastMessage(message); \
  }                                                                     \
                                                                        \
  slicer_type * vtkMRMLROS2Subscriber##name##Node::GetLastMessage(void) const \
  {                                                                     \
    vtkSmartPointer<slicer_type> result = slicer_type::New();           \
    this->GetLastMessage(result);                                       \
    return result;                                                      \
  }                                                                     \
                                                                        \
  void vtkMRMLROS2Subscriber##name##Node::GetLastMessage(vtkSmartPointer<slicer_type> message) const \
  {                                                                     \
    this->GetLastMessage(message.GetPointer());                         \
  }                                                                     \
                                                                        \
  vtkVariant vtkMRMLROS2Subscriber##name##Node::GetLastMessageVariant(void) \
  {                                                                     \
    return (reinterpret_cast<vtkMRMLROS2Subscriber##name##Internals *>(mInternals))->GetLastMessageVariant(); \
  }

#endif // __vtkMRMLROS2SubscriberMacros_h
