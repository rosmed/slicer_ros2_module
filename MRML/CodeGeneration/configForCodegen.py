# mapping from ROS to vtk/C++ types with upcast to Python wrappers are
# generated
static_type_mapping = {
    'bool': 'bool',
    'byte': 'uint8_t',
    'octet': 'uint8_t',
    'uint8': 'uint8_t',
    'int8': 'int8_t',
    'uint32': 'uint32_t',
    'int32': 'int32_t',
    'uint64': 'uint64_t',
    'int64': 'int64_t',
    'int' : 'int',
    'float': 'double',
    'double': 'double',
    'string': 'std::string',
    'str': 'std::string',
    'boolean': 'bool'
}

static_type_default_value = {
    'bool': 'false',
    'float': '0.0',
    'double': '0.0',
    'string': '""',
    'str': '""',
    'boolean': 'false'
}

vtk_equivalent_types = {
    'Pose' : 'Matrix4x4',
    'Twist': 'DoubleArray',
    'Wrench': 'DoubleArray',
    'Transform': 'Matrix4x4'
}
