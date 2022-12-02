#ifndef __vtkMRMLROS2PublisherNativeInternals_h
#define __vtkMRMLROS2PublisherNativeInternals_h

#include <vtkMRMLROS2PublisherInternals.h>

template <typename _slicer_type, typename _ros_type>
class vtkMRMLROS2PublisherNativeInternals:
  public vtkMRMLROS2PublisherTemplatedInternals<_slicer_type, _ros_type>
{
public:
  typedef vtkMRMLROS2PublisherTemplatedInternals<_slicer_type, _ros_type> BaseType;

  vtkMRMLROS2PublisherNativeInternals(vtkMRMLROS2PublisherNode * mrmlNode):
    BaseType(mrmlNode)
  {}

  _slicer_type mLastMessageSlicer;

  void Publish(const _slicer_type & msg)
  {
      vtkSlicerToROS2(msg, this->mMessageROS);
    this->mPublisher->publish(this->mMessageROS);
    // auto message = std_msgs::msg::String(); // typecasted to string - switch to templated
    // message.data = "Hello, world! " + std::to_string(this->count_++);
    // this->mPublisher->publish(message);
    this->rosNodePtr->Modified();
  }

  void TestPublisher()
  {
      // vtkSlicerToROS2(message, mMessageROS);
    // mPublisher->publish(mMessageROS);
    auto message = std_msgs::msg::String(); // typecasted to string - switch to templated
    message.data = "Hello, world! " + std::to_string(this->count_++);
    this->mPublisher->publish(message);
    this->rosNodePtr->Modified();
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
    void Publish( const slicer_type & message);			\
    void TestPublisher(void);   \
   								\
  protected:								\
    vtkMRMLROS2Publisher##name##Node();				\
    ~vtkMRMLROS2Publisher##name##Node();				\
  };


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
 void vtkMRMLROS2Publisher##name##Node::TestPublisher(void)	\
 {									\
   (dynamic_cast<vtkMRMLROS2Publisher##name##Internals *>(mInternals))->TestPublisher();					\
 }									\
									\
 void vtkMRMLROS2Publisher##name##Node::Publish(const slicer_type & message) \
 {									\
   (dynamic_cast<vtkMRMLROS2Publisher##name##Internals *>(mInternals))->Publish(message); \
 }									

#endif // __vtkMRMLROS2PublisherNativeInternals_h
