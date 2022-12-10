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

  size_t Publish(const _slicer_type & message)
  {
    const auto nbSubscriber = this->mPublisher->get_subscription_count();
    if (nbSubscriber != 0) {
      _ros_type rosMessage;
      vtkSlicerToROS2(message, rosMessage);
      this->mPublisher->publish(rosMessage);
    }
    return nbSubscriber;
  }
};

#endif // __vtkMRMLROS2PublisherNativeInternals_h
