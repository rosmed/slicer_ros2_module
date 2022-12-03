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

  void Publish(const vtkSmartPointer<_slicer_type> msg)
  {
    vtkSlicerToROS2(msg, this->mMessageROS);
    this->mPublisher->publish(this->mMessageROS);
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
    void Publish(const vtkSmartPointer<slicer_type> message);	      \
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
  void vtkMRMLROS2Publisher##name##Node::Publish(const vtkSmartPointer<slicer_type> message)  \
  {									\
    (dynamic_cast<vtkMRMLROS2Publisher##name##Internals *>(mInternals))->Publish(message); \
  }


#endif // __vtkMRMLROS2PublisherVTKInternals_h
