#include "vtkCustomTypes.h"

vtkStandardNewMacro(vtkBoolString);

vtkBoolString::vtkBoolString()
{
    result_ = false;
    message_ = "";
}

vtkBoolString::~vtkBoolString() = default;

vtkStandardNewMacro(vtkBool);

vtkBool::vtkBool()
{
    value_ = false;
}

vtkBool::~vtkBool() = default;

