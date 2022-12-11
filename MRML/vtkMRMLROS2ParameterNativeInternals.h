#ifndef __vtkMRMLROS2ParameterNativeInternals_h
#define __vtkMRMLROS2ParameterNativeInternals_h

#include <vtkMRMLROS2ParameterInternals.h>

template <typename _ros_type, typename _slicer_type>
class vtkMRMLROS2ParameterNativeInternals:
  public vtkMRMLROS2ParameterTemplatedInternals<_ros_type, _slicer_type>
{
public:
  typedef vtkMRMLROS2ParameterTemplatedInternals<_ros_type, _slicer_type> BaseType;

  vtkMRMLROS2ParameterNativeInternals(vtkMRMLROS2ParameterNode * mrmlNode):
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

#endif // __vtkMRMLROS2ParameterNativeInternals_h
