#ifndef __vtkMRMLROS2RobotNodeInternals_h
#define __vtkMRMLROS2RobotNodeInternals_h

// urdf
#include <urdf/model.h>

class vtkMRMLROS2RobotNodeInternals
{

 public:

  virtual ~vtkMRMLROS2RobotNodeInternals() = default;
  urdf::Model mURDFModel;
  std::vector< std::shared_ptr< urdf::Visual > > mVisualVector;
  std::map< std::string, std::shared_ptr< urdf::Material > > mMaterialsMap;
  std::vector< std::string> mLinkMaterials;
  std::vector< std::shared_ptr< urdf::Material > > mMaterialVector;
  std::shared_ptr<const urdf::Link> mParentLinkPointer;
  std::vector< std::shared_ptr< urdf::Link > > mChildLinkPointer;
  std::vector<urdf::Pose> mLinkOrigins;
};

#endif // __vtkMRMLROS2RobotNodeInternals_h
