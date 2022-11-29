#ifndef __vtkMRMLROS2PublisherNativeInternals_h
#define __vtkMRMLROS2PublisherNativeInternals_h

#include <vtkMRMLROS2PublisherInternals.h>

template <typename _ros_type, typename _slicer_type>
class vtkMRMLROS2PublisherNativeInternals:
  public vtkMRMLROS2PublisherTemplatedInternals<_ros_type, _slicer_type>
{
public:
  typedef vtkMRMLROS2PublisherTemplatedInternals<_ros_type, _slicer_type> BaseType;

  vtkMRMLROS2PublisherNativeInternals(vtkMRMLROS2PublisherNode * mrmlNode):
    BaseType(mrmlNode)
  {}

  _slicer_type mLastMessageSlicer;

  void GetLastMessage(_slicer_type & result)
  {
    // todo maybe add some check that we actually received a message?
    vtkROS2ToSlicer(this->mLastMessageROS, result);
  }

  vtkVariant GetLastMessageVariant(void)
  {
    GetLastMessage(mLastMessageSlicer);
    return vtkVariant(mLastMessageSlicer);
  }
};


#define VTK_MRML_ROS_PUBLISHER_NATIVE_H(slicer_type, name)		\
  class VTK_SLICER_ROS2_MODULE_LOGIC_EXPORT vtkMRMLROS2Publisher##name##Node: \
    public vtkMRMLROS2PublisherNode					\
  {									\
  public:								\
    typedef vtkMRMLROS2Publisher##name##Node SelfType;			\
    vtkTypeMacro(vtkMRMLROS2Publisher##name##Node, vtkMRMLROS2PublisherNode);	\
									\
    static SelfType * New(void);					\
    vtkMRMLNode * CreateNodeInstance(void) override;			\
    const char * GetNodeTagName(void) override;				\
    void GetLastMessage(slicer_type & message) const;			\
    vtkVariant GetLastMessageVariant(void) override;			\
									\
  protected:								\
    vtkMRMLROS2Publisher##name##Node();				\
    ~vtkMRMLROS2Publisher##name##Node();				\
  };


#define VTK_MRML_ROS_PUBLISHER_NATIVE_CXX(ros_type, slicer_type, name) \
									\
  vtkStandardNewMacro(vtkMRMLROS2Publisher##name##Node);		\
									\
  typedef vtkMRMLROS2PublisherNativeInternals<ros_type, slicer_type>	\
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
 void vtkMRMLROS2Publisher##name##Node::GetLastMessage(slicer_type & message) const \
 {									\
   (dynamic_cast<vtkMRMLROS2Publisher##name##Internals *>(mInternals))->GetLastMessage(message); \
 }									\
									\
 vtkVariant vtkMRMLROS2Publisher##name##Node::GetLastMessageVariant(void) \
 {									\
   return (dynamic_cast<vtkMRMLROS2Publisher##name##Internals *>(mInternals))->GetLastMessageVariant(); \
 }

#endif // __vtkMRMLROS2PublisherNativeInternals_h
