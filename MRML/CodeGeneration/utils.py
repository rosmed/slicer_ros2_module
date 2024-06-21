


def snake_to_camel(name):
    return ''.join(word.capitalize() for word in name.split('_'))

def camel_to_snake(name):
    return ''.join('_' + c.lower() if c.isupper() else c for c in name).lstrip('_')

def is_vtk_object(field_type, message_attribute_map):
    return '/' in field_type #and field_type in message_attribute_map

def is_vector_type(fields, static_type_mapping, message_attribute_map):
    static_types = set(static_type_mapping.get(field_type, field_type) for field_type in fields.values())
    use_vector = len(static_types) == 1 and not any(is_vtk_object(field_type, message_attribute_map) for field_type in fields.values())
    # ensure that fields is not a single field. Then a vector is not needed
    if len(fields) == 1:
        use_vector = False

    return use_vector 

def get_vtk_type(field_type, vtk_equivalent_types):
    # return True if equivalent type is found

    parts = field_type.split('/')
    if parts[-1] in vtk_equivalent_types.keys():
        return True, vtk_equivalent_types[parts[-1]]
    parts[0] = snake_to_camel(parts[0])
    field_type = parts[0] + parts[-1]


    return False, field_type