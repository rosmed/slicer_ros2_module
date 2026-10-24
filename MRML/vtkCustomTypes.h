#ifndef vtkCustomTypes_h
#define vtkCustomTypes_h

#include <vtkObject.h>
#include <vtkNew.h>
#include <string>
#include <vtkSmartPointer.h>
#include <vtkMRMLNode.h>
#include <vtkMatrix4x4.h>
#include <vtkSlicerROS2ModuleMRMLExport.h>


class vtkBoolString;
class vtkBool;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkBoolString : public vtkObject
{
public:
    vtkTypeMacro(vtkBoolString, vtkObject);
    static vtkBoolString* New();

    const bool& GetResult() const {
        return result_;
    }

    void SetResult(const bool& value) {
        result_ = value;
    }

    const std::string& GetMessage() const {
        return message_;
    }

    void SetMessage(const std::string& value) {
        message_ = value;
    }

protected:
    bool result_;
    std::string message_;

    vtkBoolString();
    ~vtkBoolString() override;

private:
    vtkBoolString(const vtkBoolString&) = delete;
    void operator=(const vtkBoolString&) = delete;
};

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkBool : public vtkObject
{
public:
    vtkTypeMacro(vtkBool, vtkObject);
    static vtkBool* New();

    const bool& GetValue() const {
        return value_;
    }

    void SetValue(const bool& value) {
        value_ = value;
    }

protected:
    bool value_;

    vtkBool();
    ~vtkBool() override;

private:
    vtkBool(const vtkBool&) = delete;
    void operator=(const vtkBool&) = delete;
};

#endif // vtkCustomTypes_h
