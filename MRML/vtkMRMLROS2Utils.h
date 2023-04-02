#ifndef __vtkMRMLROS2Utils_h
#define __vtkMRMLROS2Utils_h

#include <string>

// forward declarations
class vtkMRMLNode;
class vtkMRMLROS2NodeNode;

#include <vtkSlicerROS2ModuleMRMLExport.h>

namespace vtkMRMLROS2 {
  bool ROSInit(void);
  void ROSShutdown(void);
  vtkMRMLROS2NodeNode * CheckROS2NodeExists(vtkMRMLNode * node, const char * nodeId, std::string & errorMessage);
}

#endif // __vtkMRMLROS2Utils_h
