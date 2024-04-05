#ifndef vtkCustomTypes_h
#define vtkCustomTypes_h

#include <vtkObject.h>
#include <vtkNew.h>
#include <string>
#include <vtkSmartPointer.h>
#include <vtkMRMLNode.h>

class vtkGeometryMsgsPoseStamped;
class vtkGeometryMsgsPose;
class vtkGeometryMsgsPoint;
class vtkGeometryMsgsQuaternion;
class vtkStdMsgsHeader;
class vtkBuiltinInterfacesTime;
class vtkBoolString;
class vtkBool;

class vtkGeometryMsgsPoseStamped : public vtkObject
{
public:
    vtkTypeMacro(vtkGeometryMsgsPoseStamped, vtkObject);
    static vtkGeometryMsgsPoseStamped* New();

    vtkStdMsgsHeader* GetHeader() {
        return header_;
    }

    void SetHeader(vtkStdMsgsHeader* value) {
        header_ = value;
    }

    vtkGeometryMsgsPose* GetPose() {
        return pose_;
    }

    void SetPose(vtkGeometryMsgsPose* value) {
        pose_ = value;
    }

protected:
    vtkSmartPointer<vtkStdMsgsHeader> header_;
    vtkSmartPointer<vtkGeometryMsgsPose> pose_;

    vtkGeometryMsgsPoseStamped();
    ~vtkGeometryMsgsPoseStamped() override;
};

class vtkGeometryMsgsPose : public vtkObject
{
public:
    vtkTypeMacro(vtkGeometryMsgsPose, vtkObject);
    static vtkGeometryMsgsPose* New();

    vtkGeometryMsgsPoint* GetPosition() {
        return position_;
    }

    void SetPosition(vtkGeometryMsgsPoint* value) {
        position_ = value;
    }

    vtkGeometryMsgsQuaternion* GetOrientation() {
        return orientation_;
    }

    void SetOrientation(vtkGeometryMsgsQuaternion* value) {
        orientation_ = value;
    }

protected:
    vtkSmartPointer<vtkGeometryMsgsPoint> position_;
    vtkSmartPointer<vtkGeometryMsgsQuaternion> orientation_;

    vtkGeometryMsgsPose();
    ~vtkGeometryMsgsPose() override;
};

class vtkGeometryMsgsPoint : public vtkObject
{
public:
    vtkTypeMacro(vtkGeometryMsgsPoint, vtkObject);
    static vtkGeometryMsgsPoint* New();

    const double& GetX() const {
        return x_;
    }

    void SetX(const double& value) {
        x_ = value;
    }

    const double& GetY() const {
        return y_;
    }

    void SetY(const double& value) {
        y_ = value;
    }

    const double& GetZ() const {
        return z_;
    }

    void SetZ(const double& value) {
        z_ = value;
    }

protected:
    double x_;
    double y_;
    double z_;

    vtkGeometryMsgsPoint();
    ~vtkGeometryMsgsPoint() override;
};

class vtkGeometryMsgsQuaternion : public vtkObject
{
public:
    vtkTypeMacro(vtkGeometryMsgsQuaternion, vtkObject);
    static vtkGeometryMsgsQuaternion* New();

    const double& GetX() const {
        return x_;
    }

    void SetX(const double& value) {
        x_ = value;
    }

    const double& GetY() const {
        return y_;
    }

    void SetY(const double& value) {
        y_ = value;
    }

    const double& GetZ() const {
        return z_;
    }

    void SetZ(const double& value) {
        z_ = value;
    }

    const double& GetW() const {
        return w_;
    }

    void SetW(const double& value) {
        w_ = value;
    }

protected:
    double x_;
    double y_;
    double z_;
    double w_;

    vtkGeometryMsgsQuaternion();
    ~vtkGeometryMsgsQuaternion() override;
};

class vtkStdMsgsHeader : public vtkObject
{
public:
    vtkTypeMacro(vtkStdMsgsHeader, vtkObject);
    static vtkStdMsgsHeader* New();

    vtkBuiltinInterfacesTime* GetStamp() {
        return stamp_;
    }

    void SetStamp(vtkBuiltinInterfacesTime* value) {
        stamp_ = value;
    }

    const std::string& GetFrame_id() const {
        return frame_id_;
    }

    void SetFrame_id(const std::string& value) {
        frame_id_ = value;
    }

protected:
    vtkSmartPointer<vtkBuiltinInterfacesTime> stamp_;
    std::string frame_id_;

    vtkStdMsgsHeader();
    ~vtkStdMsgsHeader() override;
};

class vtkBuiltinInterfacesTime : public vtkObject
{
public:
    vtkTypeMacro(vtkBuiltinInterfacesTime, vtkObject);
    static vtkBuiltinInterfacesTime* New();

    const int32_t& GetSec() const {
        return sec_;
    }

    void SetSec(const int32_t& value) {
        sec_ = value;
    }

    const uint32_t& GetNanosec() const {
        return nanosec_;
    }

    void SetNanosec(const uint32_t& value) {
        nanosec_ = value;
    }

protected:
    int32_t sec_;
    uint32_t nanosec_;

    vtkBuiltinInterfacesTime();
    ~vtkBuiltinInterfacesTime() override;
};

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
