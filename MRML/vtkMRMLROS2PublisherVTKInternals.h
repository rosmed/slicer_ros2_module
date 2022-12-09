#ifndef __vtkMRMLROS2PublisherVTKInternals_h
#define __vtkMRMLROS2PublisherVTKInternals_h

#include <vtkMRMLROS2PublisherInternals.h>

template <typename _slicer_type, typename _ros_type>
class vtkMRMLROS2PublisherVTKInternals:
  public vtkMRMLROS2PublisherTemplatedInternals< _slicer_type, _ros_type>
{
public:
  typedef vtkMRMLROS2PublisherTemplatedInternals< _slicer_type, _ros_type> BaseType;

  vtkMRMLROS2PublisherVTKInternals(vtkMRMLROS2PublisherNode * mrmlNode):
    BaseType(mrmlNode)
  {
    mLastMessageSlicer = vtkNew<_slicer_type>();
  }

  vtkSmartPointer<_slicer_type> mLastMessageSlicer;

  size_t Publish(_slicer_type * msg)
  {
    const auto nbSubscriber = this->mPublisher->get_subscription_count();
    if (nbSubscriber != 0) {
      vtkSlicerToROS2(msg, this->mMessageROS);
      this->mPublisher->publish(this->mMessageROS);
    }
    return nbSubscriber;
  }
};


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
    size_t Publish(slicer_type* msg);				\
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
  size_t vtkMRMLROS2Publisher##name##Node::Publish(slicer_type * message) \
  {									\
    mNumberOfCalls++;							\
    const auto justSent = (reinterpret_cast<vtkMRMLROS2Publisher##name##Internals *>(mInternals))->Publish(message); \
    mNumberOfMessagesSent += justSent;					\
    return justSent;							\
  }									\
									\
  size_t  vtkMRMLROS2Publisher##name##Node::Publish(vtkSmartPointer<slicer_type> message) \
  {									\
    return this->Publish(message.GetPointer());				\
  }

#endif // __vtkMRMLROS2PublisherVTKInternals_h
