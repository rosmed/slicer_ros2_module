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

  size_t Publish(_slicer_type * message)
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

#endif // __vtkMRMLROS2PublisherVTKInternals_h
