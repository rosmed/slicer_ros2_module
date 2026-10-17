#ifndef __vtkAssImpConversion_h
#define __vtkAssImpConversion_h

#include <string>
#include <vector>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkSlicerROS2ModuleMRMLExport.h>

struct vtkAssImpMeshPart
{
  vtkSmartPointer<vtkPolyData> PolyData;
  std::vector<double> Color;
};

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkAssImpConversion
{
public:
  static std::vector<vtkAssImpMeshPart> vtkAssImpToPolyDataParts(const std::string& filename);
  static vtkSmartPointer<vtkPolyData> vtkAssImpToPolyData(const std::string& filename, std::vector<double>& color);
};

#endif
