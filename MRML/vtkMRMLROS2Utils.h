#ifndef __vtkMRMLROS2Utils_h
#define __vtkMRMLROS2Utils_h

#include <string>

// forward declarations
class vtkMRMLNode;
class vtkMRMLROS2NodeNode;

#include <vtkSlicerROS2ModuleMRMLExport.h>
#include <vtkMatrix4x4.h>

namespace vtkMRMLROS2 {
  inline double ToSI(const double & val) { return val / 1000.0; }
  inline double FromSI(const double & val) { return val * 1000.0; }

  inline void ToSI(vtkMatrix4x4* mat) {
    if (mat) {
      mat->SetElement(0, 3, ToSI(mat->GetElement(0, 3)));
      mat->SetElement(1, 3, ToSI(mat->GetElement(1, 3)));
      mat->SetElement(2, 3, ToSI(mat->GetElement(2, 3)));
    }
  }

  inline void FromSI(vtkMatrix4x4* mat) {
    if (mat) {
      mat->SetElement(0, 3, FromSI(mat->GetElement(0, 3)));
      mat->SetElement(1, 3, FromSI(mat->GetElement(1, 3)));
      mat->SetElement(2, 3, FromSI(mat->GetElement(2, 3)));
    }
  }

  bool ROSInit(void);
  void ROSShutdown(void);
  vtkMRMLROS2NodeNode * CheckROS2NodeExists(vtkMRMLNode * node, const char * nodeId, std::string & errorMessage);
}

#endif // __vtkMRMLROS2Utils_h
