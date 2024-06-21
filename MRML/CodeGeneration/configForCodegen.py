static_type_mapping = {
    'bool': 'bool',
    'uint32': 'uint32_t',
    'int32': 'int32_t',
    'int' : 'int32_t',
    'float': 'float',
    'double': 'double',
    'string': 'std::string',
    'str': 'std::string'
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
    'default': '0'
}

vtk_equivalent_types = {
    'Pose' : 'Matrix4x4',
    'Wrench': 'DoubleArray',
    'Transform': 'Matrix4x4'
}
