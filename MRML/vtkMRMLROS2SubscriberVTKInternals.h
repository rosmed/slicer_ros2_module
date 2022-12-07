#ifndef __vtkMRMLROS2SubscriberVTKInternals_h
#define __vtkMRMLROS2SubscriberVTKInternals_h

#include <vtkMRMLROS2SubscriberInternals.h>

template <typename _ros_type, typename _slicer_type>
class vtkMRMLROS2SubscriberVTKInternals:
  public vtkMRMLROS2SubscriberTemplatedInternals<_ros_type, _slicer_type>
{
public:
  typedef vtkMRMLROS2SubscriberTemplatedInternals<_ros_type, _slicer_type> BaseType;

  vtkMRMLROS2SubscriberVTKInternals(vtkMRMLROS2SubscriberNode * mrmlNode):
    BaseType(mrmlNode)
  {
    mLastMessageSlicer = vtkNew<_slicer_type>();
  }

  vtkSmartPointer<_slicer_type> mLastMessageSlicer;

  void GetLastMessage(vtkSmartPointer<_slicer_type> result)
  {
    // todo maybe add some check that we actually received a message?
    vtkROS2ToSlicer(this->mLastMessageROS, result);
  }

  vtkVariant GetLastMessageVariant(void)
  {
    GetLastMessage(mLastMessageSlicer);
    return vtkVariant(mLastMessageSlicer.GetPointer());
  }
};


#define VTK_MRML_ROS_SUBSCRIBER_VTK_H(slicer_type, name)		\
  class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2Subscriber##name##Node: \
    public vtkMRMLROS2SubscriberNode					\
  {									\
  public:								\
    typedef vtkMRMLROS2Subscriber##name##Node SelfType;			\
    vtkTypeMacro(vtkMRMLROS2Subscriber##name##Node, vtkMRMLROS2SubscriberNode);	\
    									\
    static SelfType * New(void);                                        \
    vtkMRMLNode * CreateNodeInstance(void) override;			\
    const char * GetNodeTagName(void) override;				\
    void GetLastMessage(vtkSmartPointer<slicer_type> message) const;	\
    vtkVariant GetLastMessageVariant(void) override;			\
    									\
  protected:								\
    vtkMRMLROS2Subscriber##name##Node();                                \
    ~vtkMRMLROS2Subscriber##name##Node();				\
  }


#define VTK_MRML_ROS_SUBSCRIBER_VTK_CXX(ros_type, slicer_type, name)	\
  									\
  vtkStandardNewMacro(vtkMRMLROS2Subscriber##name##Node);		\
  									\
  typedef vtkMRMLROS2SubscriberVTKInternals<ros_type, slicer_type>	\
  vtkMRMLROS2Subscriber##name##Internals;				\
 									\
  vtkMRMLROS2Subscriber##name##Node::vtkMRMLROS2Subscriber##name##Node() \
  {									\
    mInternals = new vtkMRMLROS2Subscriber##name##Internals(this);	\
  }									\
 									\
  vtkMRMLROS2Subscriber##name##Node::~vtkMRMLROS2Subscriber##name##Node() \
  {									\
    delete mInternals;							\
  }									\
 									\
  vtkMRMLNode * vtkMRMLROS2Subscriber##name##Node::CreateNodeInstance(void) \
  {									\
    return SelfType::New();						\
  }									\
 									\
  const char * vtkMRMLROS2Subscriber##name##Node::GetNodeTagName(void)	\
  {									\
    return "ROS2Subscriber"#name;					\
  }									\
 									\
  void vtkMRMLROS2Subscriber##name##Node::GetLastMessage(vtkSmartPointer<slicer_type> message) const \
  {									\
    (dynamic_cast<vtkMRMLROS2Subscriber##name##Internals *>(mInternals))->GetLastMessage(message); \
  }									\
 									\
  vtkVariant vtkMRMLROS2Subscriber##name##Node::GetLastMessageVariant(void) \
  {									\
    return (dynamic_cast<vtkMRMLROS2Subscriber##name##Internals *>(mInternals))->GetLastMessageVariant(); \
  }

#endif // __vtkMRMLROS2SubscriberVTKInternals_h
