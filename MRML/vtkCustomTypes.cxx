#include "vtkCustomTypes.h"

vtkStandardNewMacro(vtkGeometryMsgsPoseStamped);

vtkGeometryMsgsPoseStamped::vtkGeometryMsgsPoseStamped()
{
    header_ = vtkStdMsgsHeader::New();
    pose_ = vtkGeometryMsgsPose::New();
}

vtkGeometryMsgsPoseStamped::~vtkGeometryMsgsPoseStamped() = default;

vtkStandardNewMacro(vtkGeometryMsgsPose);

vtkGeometryMsgsPose::vtkGeometryMsgsPose()
{
    position_ = vtkGeometryMsgsPoint::New();
    orientation_ = vtkGeometryMsgsQuaternion::New();
}

vtkGeometryMsgsPose::~vtkGeometryMsgsPose() = default;

vtkStandardNewMacro(vtkGeometryMsgsPoint);

vtkGeometryMsgsPoint::vtkGeometryMsgsPoint()
{
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
}

vtkGeometryMsgsPoint::~vtkGeometryMsgsPoint() = default;

vtkStandardNewMacro(vtkGeometryMsgsQuaternion);

vtkGeometryMsgsQuaternion::vtkGeometryMsgsQuaternion()
{
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
    w_ = 0.0;
}

vtkGeometryMsgsQuaternion::~vtkGeometryMsgsQuaternion() = default;

vtkStandardNewMacro(vtkStdMsgsHeader);

vtkStdMsgsHeader::vtkStdMsgsHeader()
{
    stamp_ = vtkBuiltinInterfacesTime::New();
    frame_id_ = "";
}

vtkStdMsgsHeader::~vtkStdMsgsHeader() = default;

vtkStandardNewMacro(vtkBuiltinInterfacesTime);

vtkBuiltinInterfacesTime::vtkBuiltinInterfacesTime()
{
    sec_ = 0;
    nanosec_ = 0;
}

vtkBuiltinInterfacesTime::~vtkBuiltinInterfacesTime() = default;

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

