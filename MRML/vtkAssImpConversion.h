#ifndef __vtkAssImpConversion_h
#define __vtkAssImpConversion_h

#include <string>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

class vtkAssImpConversion
{
public:
  static vtkSmartPointer<vtkPolyData> vtkAssImpToPolyData(const std::string& filename);
};

#endif
