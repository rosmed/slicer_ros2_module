#ifndef __vtkMRMLROS2SubscriberNativeInternals_h
#define __vtkMRMLROS2SubscriberNativeInternals_h

#include <vtkMRMLROS2SubscriberInternals.h>

template <typename _ros_type, typename _slicer_type>
class vtkMRMLROS2SubscriberNativeInternals:
  public vtkMRMLROS2SubscriberTemplatedInternals<_ros_type, _slicer_type>
{
public:
  typedef vtkMRMLROS2SubscriberTemplatedInternals<_ros_type, _slicer_type> BaseType;

  vtkMRMLROS2SubscriberNativeInternals(vtkMRMLROS2SubscriberNode * mrmlNode):
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

#endif // __vtkMRMLROS2SubscriberNativeInternals_h
