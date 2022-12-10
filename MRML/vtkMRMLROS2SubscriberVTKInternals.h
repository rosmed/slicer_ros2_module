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

  void GetLastMessage(_slicer_type * result)
  {
    // todo maybe add some check that we actually received a message?
    vtkROS2ToSlicer(this->mLastMessageROS, result);
  }

  vtkVariant GetLastMessageVariant(void)
  {
    GetLastMessage(mLastMessageSlicer.GetPointer());
    return vtkVariant(mLastMessageSlicer.GetPointer());
  }
};

#endif // __vtkMRMLROS2SubscriberVTKInternals_h
