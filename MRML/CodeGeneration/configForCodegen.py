# mapping from ROS to vtk/C++ types with upcast to Python wrappers are
# generated
# static_type_mapping = {
#     'bool': 'bool',
#     'byte': 'uint8_t',
#     'octet': 'uint8_t',
#     'uint8': 'uint8_t',
#     'int8': 'int8_t',
#     'uint32': 'uint32_t',
#     'int32': 'int32_t',
#     'uint64': 'uint64_t',
#     'int64': 'int64_t',
#     'int' : 'int',
#     'float': 'double',
#     'double': 'double',
#     'string': 'std::string',
#     'str': 'std::string',
#     'boolean': 'bool'
# }

# static_type_default_value = {
#     'bool': 'false',
#     'float': '0.0',
#     'double': '0.0',
#     'string': '""',
#     'str': '""',
#     'boolean': 'false'
# }

# vtk_equivalent_types = {
#     'Pose' : 'Matrix4x4',
#     'Twist': 'DoubleArray',
#     'Wrench': 'DoubleArray',
#     'Transform': 'Matrix4x4'
# }



# mapping from ROS to vtk/C++ types with upcast to Python wrappers are
# generated
static_type_mapping = {
    'bool': 'bool',
    'uint32': 'int',
    'int32': 'int',
    'int' : 'int',
    'float': 'double',
    'double': 'double',
    'string': 'std::string',
    'str': 'std::string',
    'boolean': 'bool',

    'byte': 'int',
    'octet': 'int',
    'char': 'int',
    'float32': 'double',
    'float64': 'double',
    'int8': 'int',
    'uint8': 'int',
    'int16': 'int',
    'uint16': 'int',
    'int64': 'int',
    'uint64': 'int',
    'wstring': 'std::string',
    'string<22>': 'std::string'

    
}

static_type_default_value = {
    'bool': 'false',
    'uint32': '0',
    'int32': '0',
    'int': '0',
    'float': '0.0',
    'double': '0.0',
    'string': '""',
    'str': '""',
    'default': '0',
    'boolean': 'false',

        'byte': '0',
    'octet': '0',
    'char': '0',
    'float32': '0.0',
    'float64': '0.0',
    'int8': '0',
    'uint8': '0',
    'int16': '0',
    'uint16': '0',
    'int64': '0',
    'uint64': '0',
    'wstring': '""',
    'string<22>': '""'
}

vtk_equivalent_types = {
    'Pose' : 'Matrix4x4',
    'Twist': 'DoubleArray',
    'Wrench': 'DoubleArray',
    'Transform': 'Matrix4x4'
}
