#!/usr/bin/python3

import argparse
import sys

import rclpy

# v2 dependencies
import importlib
import json
from rosidl_runtime_py import message_to_ordereddict
from configForCodegen import static_type_mapping, static_type_default_value, vtk_equivalent_types
from utils import snake_to_camel, camel_to_snake, is_vtk_object, is_vector_type, get_vtk_type


################################################################33

def get_class_name_formatted(class_name):
    """
    As an example, the class name 'geometry_msgs/msg/PoseStamped',
    the name of the class will be 'GeometryMsgsPoseStamped' <- class_name_formatted
    but similar data types would be identified using PoseStamped <- class_type_identifier
    """

    [package, namespace, message] = class_name.split('/')
    class_type_identifier = message
    class_name_formatted = snake_to_camel(package) + message
    return class_name_formatted, class_type_identifier

def generate_vector_code(class_code_hpp, class_code_cpp, class_name, fields):

    class_code_cpp += f"    data_.resize({len(fields)});\n"

    vector_type = list(fields.values())[0]

    class_code_hpp += f"    const std::vector<{vector_type}>& Get{class_name}Vector() const {{\n"
    class_code_hpp += f"        return data_;\n"
    class_code_hpp += f"    }}\n\n"
    class_code_hpp += f"    void Set{class_name}Vector(const std::vector<{vector_type}>& value) {{\n"
    class_code_hpp += f"        data_ = value;\n"
    class_code_hpp += f"    }}\n\n"

    class_code_hpp += "protected:\n"
    class_code_hpp += f"    std::vector<{vector_type}> data_;\n"

    return class_code_hpp, class_code_cpp



def generate_vtk_getset(method_code_hpp, field_name, field_type):

    method_code_hpp += f"    vtk{field_type}* Get{field_name.capitalize()}() {{\n"
    method_code_hpp += f"        return {field_name}_;\n"
    method_code_hpp += f"    }}\n\n"
    method_code_hpp += f"    void Set{field_name.capitalize()}(vtk{field_type}* value) {{\n"
    method_code_hpp += f"        {field_name}_ = value;\n"
    method_code_hpp += f"    }}\n\n"

    return method_code_hpp

def generate_static_getset(method_code_hpp, field_name, field_type, static_type_mapping):
    method_code_hpp += f"    const {static_type_mapping[field_type]}& Get{field_name.capitalize()}() const {{\n"
    method_code_hpp += f"        return {field_name}_;\n"
    method_code_hpp += f"    }}\n\n"
    method_code_hpp += f"    void Set{field_name.capitalize()}(const {static_type_mapping[field_type]}& value) {{\n"
    method_code_hpp += f"        {field_name}_ = value;\n"
    method_code_hpp += f"    }}\n\n"

    return method_code_hpp

def generate_static_sequence_getset(method_code_hpp, field_name, field_type, static_type_mapping):
    # method_code_hpp += f"    const {static_type_mapping[field_type]}* Get{field_name.capitalize()}Vector() const {{\n"
    # method_code_hpp += f"        return {field_name}_;\n"
    # method_code_hpp += f"    }}\n\n"

    method_code_hpp += f"    const std::vector<{static_type_mapping[field_type]}>& Get{field_name.capitalize()}Vector() const {{\n"
    method_code_hpp += f"        return {field_name}_;\n"
    method_code_hpp += f"    }}\n\n"

    # method_code_hpp += f"    void Set{field_name.capitalize()}Vector(const {static_type_mapping[field_type]}* value) {{\n"
    # method_code_hpp += f"        {field_name}_ = value;\n"
    # method_code_hpp += f"    }}\n\n"

    method_code_hpp += f"    void Set{field_name.capitalize()}Vector(const std::vector<{static_type_mapping[field_type]}>& value) {{\n"
    method_code_hpp += f"        {field_name}_ = value;\n"
    method_code_hpp += f"    }}\n\n"

    return method_code_hpp

def generate_class(class_name, fields, message_attribute_map):

    # Create the class code for the .cpp file
    class_code_cpp = f"vtkStandardNewMacro(vtk{class_name});\n\n"
    class_code_cpp += f"vtk{class_name}::vtk{class_name}()\n{{\n"

    # Create the class code for the .hpp file
    class_code_hpp = f"class vtk{class_name} : public vtkObject\n{{\npublic:\n"
    class_code_hpp += f"    vtkTypeMacro(vtk{class_name}, vtkObject);\n"
    class_code_hpp += f"    static vtk{class_name}* New();\n\n"

    use_vector = is_vector_type(fields, static_type_mapping, message_attribute_map)
    
    if use_vector: ## This is for cases where all the fields are of the same type
        class_code_hpp, class_code_cpp = generate_vector_code(class_code_hpp, class_code_cpp, class_name, fields)

    else:

        method_code_hpp = ""
        attribute_code_hpp = ""
        attribute_code_cpp = ""

        for field_name, field_type in fields.items():
            # Check if the field is a VTK object or not 
            if is_vtk_object(field_type, message_attribute_map):
                is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
                method_code_hpp = generate_vtk_getset(method_code_hpp, field_name, field_type)
                attribute_code_hpp += f"    vtkSmartPointer<vtk{field_type}> {field_name}_;\n"
                attribute_code_cpp += f"    {field_name}_ = vtk{field_type}::New();\n"

            elif "sequence" in field_type:
                static_type = field_type.split('<')[1].split('>')[0]
                method_code_hpp = generate_static_sequence_getset(method_code_hpp, field_name, static_type, static_type_mapping)
                # attribute_code_hpp += f"    {static_type_mapping[static_type]}* {field_name}_;\n"
                attribute_code_hpp += f"    std::vector<{static_type_mapping[static_type]}> {field_name}_;\n"

            else:
                method_code_hpp = generate_static_getset(method_code_hpp, field_name, field_type, static_type_mapping)
                attribute_code_hpp += f"    {static_type_mapping[field_type]} {field_name}_;\n" 
                attribute_code_cpp += f"    {field_name}_ = {static_type_default_value.get(field_type, 'default')};\n"


        class_code_hpp += method_code_hpp
        class_code_hpp += "protected:\n"
        class_code_hpp += attribute_code_hpp
        class_code_cpp += attribute_code_cpp

    class_code_cpp += "}\n\n"
    class_code_cpp += f"vtk{class_name}::~vtk{class_name}() = default;\n\n"

    class_code_hpp += f"\n    vtk{class_name}();\n"
    class_code_hpp += f"    ~vtk{class_name}() override;\n"
    class_code_hpp += "};\n\n"

    return class_code_hpp, class_code_cpp

def identify_imports(class_name, namespace, package_name, fields, message_attribute_map):
    imports = ""
    # usual suspects
    imports += "#include <vtkObject.h>\n"
    imports += "#include <vtkNew.h>\n"
    imports += "#include <string>\n"
    imports += "#include <vtkSmartPointer.h>\n"
    imports += "#include <vtkMRMLNode.h>\n\n"

    # include the message type
    imports += f"#include <rclcpp/rclcpp.hpp>\n"
    imports += f"#include <{package_name}/{namespace}/{camel_to_snake(class_name)}.hpp>\n"

    # include vtkROS2ToSlicer and vtkSlicerToROS2
    imports += f"#include \"vtkROS2ToSlicer.h\"\n"
    imports += f"#include \"vtkSlicerToROS2.h\"\n"

    for field_name, field_type in fields.items():
        if is_vtk_object(field_type, message_attribute_map):
            is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
            if not is_equivalent_type_available:
                imports+=f"#include \"vtkROS2{field_type}.h\"\n"
            else:
                imports += f"#include \"vtk{field_type}.h\"\n"
    return imports


def generate_slicer_to_ros2_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields, message_attribute_map):
    code_string_cpp = "\n"
    code_string_hpp = ""

    class_name_formatted = vtk_equivalent_types.get(class_type_identifier, class_name_formatted)
    code_string_cpp += f"void vtkSlicerToROS2( vtk{class_name_formatted} * input , {msg_ros2_type} & result, const std::shared_ptr<rclcpp::Node> & rosNode) {{\n "
    code_string_hpp += f"void vtkSlicerToROS2( vtk{class_name_formatted} * input , {msg_ros2_type} & result, const std::shared_ptr<rclcpp::Node> & rosNode);\n"

    use_vector = is_vector_type(fields, static_type_mapping, message_attribute_map)
    if use_vector:
        idx = 0
        for field_name, field_type in fields.items():
            code_string_cpp += f"\tresult.{field_name} = input->Get{class_name_formatted}Vector()[{idx}];\n"
            idx += 1
        
    else:
        for field_name, field_type in fields.items():
            if is_vtk_object(field_type, message_attribute_map):
                is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
                code_string_cpp += f"\tvtkSlicerToROS2(input->Get{field_name.capitalize()}(), result.{field_name}, rosNode);\n"
            elif "sequence" in field_type:
                static_type = field_type.split('<')[1].split('>')[0]
                code_string_cpp += f"\tresult.{field_name} = input->Get{field_name.capitalize()}Vector();\n"
            else:
                code_string_cpp += f"\tresult.{field_name} = input->Get{field_name.capitalize()}();\n"

    code_string_cpp += "}\n\n"
    return code_string_hpp, code_string_cpp

def generate_ros2_to_slicer_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields, message_attribute_map):
    code_string_cpp = "\n"
    code_string_hpp = ""
    class_name_formatted = vtk_equivalent_types.get(class_type_identifier, class_name_formatted)
    code_string_hpp += f"void vtkROS2ToSlicer( const {msg_ros2_type} & input , vtkSmartPointer<vtk{class_name_formatted}> result);\n"
    code_string_cpp += f"void vtkROS2ToSlicer( const {msg_ros2_type} & input , vtkSmartPointer<vtk{class_name_formatted}> result) {{\n "

    use_vector = is_vector_type(fields, static_type_mapping, message_attribute_map)
    if use_vector:
        vector_type = list(fields.values())[0]
        code_string_cpp += f"\tstd::vector<{vector_type}> data;\n"
        for field_name, field_type in fields.items():
            code_string_cpp += f"\tdata.push_back(input.{field_name});\n"
        code_string_cpp += f"\tresult->Set{class_name_formatted}Vector(data);\n"

    else:
        for field_name, field_type in fields.items():
            if is_vtk_object(field_type, message_attribute_map):
                is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
                code_string_cpp += f"\tvtkSmartPointer<vtk{field_type}> {field_name} = vtkSmartPointer<vtk{field_type}>::New();\n"
                code_string_cpp += f"\tvtkROS2ToSlicer(input.{field_name}, {field_name});\n"
                code_string_cpp += f"\tresult->Set{field_name.capitalize()}({field_name});\n"
            elif "sequence" in field_type:
                static_type = field_type.split('<')[1].split('>')[0]
                code_string_cpp += f"\tresult->Set{field_name.capitalize()}Vector(input.{field_name});\n"
            else:
                code_string_cpp += f"\tresult->Set{field_name.capitalize()}(input.{field_name});\n"

    code_string_cpp += "}\n\n"
    return code_string_hpp, code_string_cpp

def generate_message_dict_v2(message_type): ## TODO: Refactor this function
    [package_name, namespace, msg_name] = message_type.split('/')

    msg_package = importlib.import_module(f"{package_name}.msg")
    msg_attributes = getattr(msg_package, msg_name)

    attributes = msg_attributes.get_fields_and_field_types()

    msg_dict = {}
    msg_dict[message_type] = {}

    prefix = message_type

    for attribute, attribute_type in attributes.items():
        if '/' in attribute_type:
            [package_name, msg_name] = attribute_type.split('/')
            sub_message_type = f"{package_name}/{namespace}/{msg_name}"
            msg_dict[prefix][attribute] = sub_message_type
        else:
            msg_dict[prefix][attribute] = attribute_type

    return msg_dict


def ROS2_to_vtkObject_v2(_message, _directory):
    [package, namespace, msg_name] = _message.split('/')
    print(f"Generating code for message: {_message}")

    message_attribute_map = generate_message_dict_v2(_message)

    fields = message_attribute_map[_message]

    # for class_name, fields in message_attribute_map.items():
    class_name_formatted, class_type_identifier = get_class_name_formatted(_message)
    if class_type_identifier in vtk_equivalent_types.keys():
        return

    hpp_code = ""
    cpp_code = ""
    class_definitions_code_hpp = ""
    class_definitions_code_cpp = ""

    filename = f"vtkROS2{class_name_formatted}"
    hpp_code += f"#ifndef {filename}_h\n"
    hpp_code += f"#define {filename}_h\n\n"

    cpp_code += f"#include \"{filename}.h\"\n\n"

    imports = identify_imports(msg_name, namespace, package, fields, message_attribute_map,)
    hpp_code += imports
    hpp_code += f"// Forward declarations\n"

    # forward declarations
    hpp_code += f"class vtk{class_name_formatted};\n"
    class_code_hpp_single, class_code_cpp_single = generate_class(class_name_formatted, fields, message_attribute_map)
    class_definitions_code_hpp += class_code_hpp_single
    class_definitions_code_cpp += class_code_cpp_single

    vtk_equivalent_types[class_type_identifier] = class_name_formatted

    hpp_code += "\n"
    hpp_code += class_definitions_code_hpp
    cpp_code += class_definitions_code_cpp

    # Add Slicer to ROS2 conversion functions and vice versa
    hpp_code += f"// Conversion functions\n"
    
    msg_ros2_type = f"{package}::{namespace}::{msg_name}"
    hpp_code_single, cpp_code_single = generate_slicer_to_ros2_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields, message_attribute_map)
    hpp_code += hpp_code_single
    cpp_code += cpp_code_single

    hpp_code_single, cpp_code_single = generate_ros2_to_slicer_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields, message_attribute_map)
    hpp_code += hpp_code_single
    cpp_code += cpp_code_single
    

    hpp_code += f"\n#endif // {filename}_h\n"

    
    with open(_directory + '/' + filename + '.h', 'w') as h:
        h.write(hpp_code)

    with open(_directory + '/' + filename + '.cxx', 'w') as cxx:
        cxx.write(cpp_code)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--message', type = str, required=True,
                        help = 'ROS message type.  For example \"geometry_msgs/msg/PoseStamped\"')
    parser.add_argument('-c', '--class-name', type = str, required = True)
    parser.add_argument('-d', '--directory', type = str, required = True)
    args = parser.parse_args(sys.argv[1:])
    application = ROS2_to_vtkObject_v2(args.message,  args.directory)
