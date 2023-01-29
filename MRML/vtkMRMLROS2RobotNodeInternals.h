#ifndef __vtkMRMLROS2RobotNodeInternals_h
#define __vtkMRMLROS2RobotNodeInternals_h

// urdf
#include <urdf/model.h>

class vtkMRMLROS2RobotNodeInternals
{

 public:

  virtual ~vtkMRMLROS2RobotNodeInternals() = default;
  urdf::Model mModel;
  std::vector< std::shared_ptr< urdf::Visual > > mVisualVector;
  std::shared_ptr<const urdf::Link> mParentLinkPointer;
  std::vector< std::shared_ptr< urdf::Link > > mChildLinkPointer;
  std::vector<urdf::Pose> mLinkOrigins;
};

#endif // __vtkMRMLROS2RobotNodeInternals_h
