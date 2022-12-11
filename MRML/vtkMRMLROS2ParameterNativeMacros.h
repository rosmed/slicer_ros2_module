#ifndef __vtkMRMLROS2ParameterNativeMacros_h
#define __vtkMRMLROS2ParameterNativeMacros_h

#define VTK_MRML_ROS_PARAMETER_NATIVE_H(slicer_type, name)		\
  class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2Parameter##name##Node: \
    public vtkMRMLROS2ParameterNode					\
  {									\
  public:								\
    typedef vtkMRMLROS2Parameter##name##Node SelfType;			\
    vtkTypeMacro(vtkMRMLROS2Parameter##name##Node, vtkMRMLROS2ParameterNode);	\
									\
    static SelfType * New(void);					\
    vtkMRMLNode * CreateNodeInstance(void) override;			\
    const char * GetNodeTagName(void) override;				\
    void GetLastMessage(slicer_type & message) const;			\
    vtkVariant GetLastMessageVariant(void) override;			\
									\
  protected:								\
    vtkMRMLROS2Parameter##name##Node();				\
    ~vtkMRMLROS2Parameter##name##Node();				\
  }


#define VTK_MRML_ROS_PARAMETER_NATIVE_CXX(ros_type, slicer_type, name) \
									\
  vtkStandardNewMacro(vtkMRMLROS2Parameter##name##Node);		\
									\
  typedef vtkMRMLROS2ParameterNativeInternals<ros_type, slicer_type>	\
  vtkMRMLROS2Parameter##name##Internals;				\
  									\
 vtkMRMLROS2Parameter##name##Node::vtkMRMLROS2Parameter##name##Node()	\
 {									\
   mInternals = new vtkMRMLROS2Parameter##name##Internals(this);	\
 }									\
									\
 vtkMRMLROS2Parameter##name##Node::~vtkMRMLROS2Parameter##name##Node() \
 {									\
   delete mInternals;							\
 }									\
									\
 vtkMRMLNode * vtkMRMLROS2Parameter##name##Node::CreateNodeInstance(void) \
 {									\
   return SelfType::New();						\
 }									\
									\
 const char * vtkMRMLROS2Parameter##name##Node::GetNodeTagName(void)	\
 {									\
   return "ROS2Parameter"#name;					\
 }									\
									\
 void vtkMRMLROS2Parameter##name##Node::GetLastMessage(slicer_type & message) const \
 {									\
   (reinterpret_cast<vtkMRMLROS2Parameter##name##Internals *>(mInternals))->GetLastMessage(message); \
 }									\
									\
 vtkVariant vtkMRMLROS2Parameter##name##Node::GetLastMessageVariant(void) \
 {									\
   return (reinterpret_cast<vtkMRMLROS2Parameter##name##Internals *>(mInternals))->GetLastMessageVariant(); \
 }

#endif // __vtkMRMLROS2ParameterNativeMacros_h
