

def snake_to_camel(name):
    return ''.join(word.capitalize() for word in name.split('_'))


def camel_to_snake(name):
    return ''.join('_' + c.lower() if c.isupper() else c for c in name).lstrip('_')


def is_vtk_object(field_type, message_attribute_map):
    return '/' in field_type #and field_type in message_attribute_map


def get_vtk_type(field_type, vtk_equivalent_types):
    # return True if equivalent type is found

    parts = field_type.split('/')
    if parts[-1] in vtk_equivalent_types.keys():
        return True, vtk_equivalent_types[parts[-1]]
    parts[0] = snake_to_camel(parts[0])
    field_type = parts[0] + parts[-1]

    return False, field_type
