# mapping from ROS to vtk/C++ types with upcast to Python wrappers are
# generated
ros2_to_cpp_type_static_mapping = {
    # Boolean types
    'bool': 'bool',
    'boolean': 'bool',

    # Fixed-width integer types
    'int8': 'char',
    'uint8': 'unsigned char',
    'byte': 'unsigned char',    # ROS2 byte is unsigned
    'octet': 'unsigned char',   # ROS2 octet is unsigned
    'char': 'char',       # Changed from int to char
    'int16': 'int',
    'uint16': 'unsigned int',
    'int32': 'int',
    'uint32': 'unsigned int',
    'int64': 'long int',
    'uint64': 'unsigned long int',
    'int': 'int',     # ROS2 int defaults to 32-bit

    # Floating point types
    'float32': 'float',   # Changed from double to float for precision match
    'float64': 'double',
    'float': 'float',     # Changed from double to float
    'double': 'double',

    # String types
    'string': 'std::string',
    'str': 'std::string',
    'wstring': 'std::wstring',  # Changed to wide string
    'string<22>': 'std::string' # I have no idea why this is a type.  ref: https://docs.ros2.org/foxy/api/test_msgs/msg/Strings.html
}

static_cpp_type_default_value = {
    # Boolean types
    'bool': 'false',
    'boolean': 'false',

    # Fixed-width integer types
    'int8': '0',
    'uint8': '0',
    'byte': '0',
    'octet': '0',
    'char': '0',
    'int16': '0',
    'uint16': '0',
    'int32': '0',
    'uint32': '0',
    'int64': '0',
    'uint64': '0',
    'int': '0',

    # Floating point types
    'float32': '0.0',
    'float64': '0.0',
    'float': '0.0',
    'double': '0.0',

    # String types
    'string': '""',
    'str': '""',
    'wstring': '""',
    'string<22>': '""',

    # Default value
    'default': '0'
}

vtk_equivalent_types = {
    'Pose' : 'Matrix4x4',
    'Twist': 'DoubleArray',
    'Wrench': 'DoubleArray',
    'Transform': 'Matrix4x4'
}

vtk_ignored_types = {
    # 'geometry_msgs/msg/Pose': True,
}
