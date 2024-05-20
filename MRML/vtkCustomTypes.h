#ifndef vtkCustomTypes_h
#define vtkCustomTypes_h

#include <vtkObject.h>
#include <vtkNew.h>
#include <string>
#include <vtkSmartPointer.h>
#include <vtkMRMLNode.h>
#include <vtkMatrix4x4.h>


class vtkBoolString;
class vtkBool;

class vtkBoolString : public vtkObject
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
};

class vtkBool : public vtkObject
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
};

#endif // vtkCustomTypes_h
