#include "vtkCustomTypes.h"

vtkStandardNewMacro(vtkGeometryMsgsPoseStamped);

vtkGeometryMsgsPoseStamped::vtkGeometryMsgsPoseStamped()
{
    header_ = vtkStdMsgsHeader::New();
    pose_ = vtkMatrix4x4::New();
}

vtkGeometryMsgsPoseStamped::~vtkGeometryMsgsPoseStamped() = default;

vtkStandardNewMacro(vtkGeometryMsgsPoint);

vtkGeometryMsgsPoint::vtkGeometryMsgsPoint()
{
    data_.resize(3);
}

vtkGeometryMsgsPoint::~vtkGeometryMsgsPoint() = default;

vtkStandardNewMacro(vtkGeometryMsgsQuaternion);

vtkGeometryMsgsQuaternion::vtkGeometryMsgsQuaternion()
{
    data_.resize(4);
}

vtkGeometryMsgsQuaternion::~vtkGeometryMsgsQuaternion() = default;

vtkStandardNewMacro(vtkBuiltinInterfacesTime);

vtkBuiltinInterfacesTime::vtkBuiltinInterfacesTime()
{
    sec_ = 0;
    nanosec_ = 0;
}

vtkBuiltinInterfacesTime::~vtkBuiltinInterfacesTime() = default;

vtkStandardNewMacro(vtkStdMsgsHeader);

vtkStdMsgsHeader::vtkStdMsgsHeader()
{
    stamp_ = vtkBuiltinInterfacesTime::New();
    frame_id_ = "";
}

vtkStdMsgsHeader::~vtkStdMsgsHeader() = default;

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

