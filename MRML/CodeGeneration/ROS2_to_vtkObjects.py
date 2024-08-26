#!/usr/bin/python3

import argparse
import sys

import rclpy

# v2 dependencies
import importlib
import json
from rosidl_runtime_py import message_to_ordereddict
from configForCodegen import static_type_mapping, static_type_default_value, vtk_equivalent_types
from utils import snake_to_camel, camel_to_snake, is_vtk_object, get_vtk_type

__ = " "
____ = "   "

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


def generate_vtk_getset(method_code_hpp, field_name, field_type):
    method_code_hpp += f"{__} // generate_vtk_getset\n"
    method_code_hpp += f"{__} vtk{field_type} * Get{field_name.capitalize()}(void) {{\n"
    method_code_hpp += f"{____} return {field_name}_;\n"
    method_code_hpp += f"{__} }}\n\n"
    method_code_hpp += f"{__} void Set{field_name.capitalize()}(vtk{field_type} * value) {{\n"
    method_code_hpp += f"{____} {field_name}_ = value;\n"
    method_code_hpp += f"{__} }}\n\n"

    return method_code_hpp


def generate_static_getset(method_code_hpp, field_name, field_type, static_type_mapping):
    method_code_hpp += f"{__}// generate_static_getset\n"
    method_code_hpp += f"{__} const {static_type_mapping[field_type]}& Get{field_name.capitalize()}(void) const {{\n"
    method_code_hpp += f"{____} return {field_name}_;\n"
    method_code_hpp += f"{__} }}\n\n"
    method_code_hpp += f"{__} void Set{field_name.capitalize()}(const {static_type_mapping[field_type]}& value) {{\n"
    method_code_hpp += f"{____} {field_name}_ = value;\n"
    method_code_hpp += f"{__} }}\n\n"

    return method_code_hpp


def generate_static_sequence_getset(method_code_hpp, field_name, field_type, static_type_mapping):
    # method_code_hpp += f"    const {static_type_mapping[field_type]}* Get{field_name.capitalize()}(void) const {{\n"
    # method_code_hpp += f"        return {field_name}_;\n"
    # method_code_hpp += f"    }}\n\n"

    method_code_hpp += f"    const std::vector<{static_type_mapping[field_type]}>& Get{field_name.capitalize()}(void) const {{\n"
    method_code_hpp += f"        return {field_name}_;\n"
    method_code_hpp += f"    }}\n\n"

    # method_code_hpp += f"    void Set{field_name.capitalize()}(const {static_type_mapping[field_type]}* value) {{\n"
    # method_code_hpp += f"        {field_name}_ = value;\n"
    # method_code_hpp += f"    }}\n\n"

    method_code_hpp += f"    void Set{field_name.capitalize()}(const std::vector<{static_type_mapping[field_type]}>& value) {{\n"
    method_code_hpp += f"        {field_name}_ = value;\n"
    method_code_hpp += f"    }}\n\n"

    return method_code_hpp


def generate_class(class_name, fields):

    # Create the class code for the .cpp file
    class_code_cpp = "// generate_class\n"
    class_code_cpp += f"vtkStandardNewMacro(vtk{class_name});\n\n"
    class_code_cpp += f"vtk{class_name}::vtk{class_name}()\n{{\n"

    # Create the class code for the .hpp file
    class_code_hpp = "// generate_class\n"
    class_code_hpp += f"class vtk{class_name} : public vtkObject\n{{\npublic:\n"
    class_code_hpp += f"{__} vtkTypeMacro(vtk{class_name}, vtkObject);\n"
    class_code_hpp += f"{__} static vtk{class_name}* New(void);\n\n"

    method_code_hpp = ""
    attribute_code_hpp = ""
    attribute_code_cpp = ""

    for field_name, field_type in fields.items():
        # Check if the field is a VTK object or not
        if is_vtk_object(field_type):
            is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
            method_code_hpp = generate_vtk_getset(method_code_hpp, field_name, field_type)
            attribute_code_hpp += f"{__} vtkSmartPointer<vtk{field_type}> {field_name}_;\n"
            attribute_code_cpp += f"{__} {field_name}_ = vtk{field_type}::New();\n"

        elif "sequence" in field_type:
            static_type = field_type.split('<')[1].split('>')[0]
            method_code_hpp = generate_static_sequence_getset(method_code_hpp, field_name, static_type, static_type_mapping)
            # attribute_code_hpp += f"    {static_type_mapping[static_type]}* {field_name}_;\n"
            attribute_code_hpp += f"{__} std::vector<{static_type_mapping[static_type]}> {field_name}_;\n"

        else:
            method_code_hpp = generate_static_getset(method_code_hpp, field_name, field_type, static_type_mapping)
            attribute_code_hpp += f"{__} {static_type_mapping[field_type]} {field_name}_;\n"
            attribute_code_cpp += f"{__} {field_name}_ = {static_type_default_value.get(field_type, 'default')};\n"

    class_code_hpp += method_code_hpp
    class_code_hpp += "protected:\n"
    class_code_hpp += attribute_code_hpp
    class_code_cpp += attribute_code_cpp

    class_code_cpp += "}\n\n"
    class_code_cpp += f"vtk{class_name}::~vtk{class_name}() = default;\n\n"

    class_code_hpp += "\n"
    class_code_hpp += f"{__} vtk{class_name}();\n"
    class_code_hpp += f"{__} ~vtk{class_name}() override;\n"
    class_code_hpp += "};\n\n"

    return class_code_hpp, class_code_cpp

def identify_imports(class_name, namespace, package_name, attribute_list):
    imports = "// identify_imports\n"
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
    imports += f"#include <vtkROS2ToSlicer.h>\n"
    imports += f"#include <vtkSlicerToROS2.h>\n"

    for field_type in attribute_list:
        if is_vtk_object(field_type):
            is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
            if not is_equivalent_type_available:
                imports+=f"#include <vtk{field_type}.h>\n"
            else:
                imports += f"#include <vtk{field_type}.h>\n"
    return imports


def generate_slicer_to_ros2_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields):
    code_string_cpp = "\n// generate_slicer_to_ros2_methods_for_class\n"
    code_string_hpp = "// generate_slicer_to_ros2_methods_for_class\n"

    class_name_formatted = vtk_equivalent_types.get(class_type_identifier, class_name_formatted)
    code_string_cpp += f"void vtkSlicerToROS2( vtk{class_name_formatted} * input, {msg_ros2_type} & result, const std::shared_ptr<rclcpp::Node> & rosNode) {{\n"
    code_string_hpp += f"void vtkSlicerToROS2( vtk{class_name_formatted} * input, {msg_ros2_type} & result, const std::shared_ptr<rclcpp::Node> & rosNode);\n"

    for field_name, field_type in fields.items():
        if is_vtk_object(field_type):
            is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
            code_string_cpp += f"{__} vtkSlicerToROS2(input->Get{field_name.capitalize()}(), result.{field_name}, rosNode);\n"
        elif "sequence" in field_type:
            code_string_cpp += f"{__} result.{field_name}.resize(input->Get{field_name.capitalize()}().size());\n"
            code_string_cpp += f"{__} std::copy(input->Get{field_name.capitalize()}().begin(), input->Get{field_name.capitalize()}().end(), result.{field_name}.begin());\n"
        else:
            code_string_cpp += f"{__} result.{field_name} = input->Get{field_name.capitalize()}();\n"

    code_string_cpp += "}\n\n"
    return code_string_hpp, code_string_cpp


def generate_ros2_to_slicer_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields):
    code_string_cpp = "\n// generate_ros2_to_slicer_methods_for_class\n"
    code_string_hpp = "// generate_ros2_to_slicer_methods_for_class\n"
    class_name_formatted = vtk_equivalent_types.get(class_type_identifier, class_name_formatted)
    code_string_hpp += f"void vtkROS2ToSlicer(const {msg_ros2_type} & input, vtkSmartPointer<vtk{class_name_formatted}> result);\n"
    code_string_cpp += f"void vtkROS2ToSlicer(const {msg_ros2_type} & input, vtkSmartPointer<vtk{class_name_formatted}> result) {{\n"

    for field_name, field_type in fields.items():
        if is_vtk_object(field_type):
            is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
            code_string_cpp += f"{__} vtkSmartPointer<vtk{field_type}> {field_name} = vtkSmartPointer<vtk{field_type}>::New();\n"
            code_string_cpp += f"{__} vtkROS2ToSlicer(input.{field_name}, {field_name});\n"
            code_string_cpp += f"{__} result->Set{field_name.capitalize()}({field_name});\n"
        elif "sequence" in field_type:
            is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
            element_type = field_type.split('<')[1].split('>')[0]
            code_string_cpp += f"{__} std::vector<{static_type_mapping[element_type]}> temp_{field_name};\n"
            code_string_cpp += f"{__} temp_{field_name}.resize(input.{field_name}.size());\n"
            code_string_cpp += f"{__} std::copy(input.{field_name}.begin(), input.{field_name}.end(), temp_{field_name}.begin());\n"
            code_string_cpp += f"{__} result->Set{field_name.capitalize()}(temp_{field_name});\n"
        else:
            code_string_cpp += f"{__} result->Set{field_name.capitalize()}(input.{field_name});\n"

    code_string_cpp += "}\n\n"
    return code_string_hpp, code_string_cpp

# def generate_attribute_dict(full_type_name, is_service=False):
#     [package_name, namespace, type_name] = full_type_name.split('/')
    
#     if is_service:
#         package = importlib.import_module(f"{package_name}.srv")
#         attributes = getattr(package, type_name)
#         request_attributes = attributes.Request.get_fields_and_field_types()
#         response_attributes = attributes.Response.get_fields_and_field_types()
        
#         result = {
#             full_type_name: {
#                 'request': process_attributes(request_attributes, namespace),
#                 'response': process_attributes(response_attributes, namespace)
#             }
#         }
#         return result, set(list(request_attributes.values()) + list(response_attributes.values()))
#     else:
#         package = importlib.import_module(f"{package_name}.msg")
#         attributes = getattr(package, type_name).get_fields_and_field_types()
        
#         return {full_type_name: process_attributes(attributes, namespace)}, list(attributes.values())
    
def generate_attribute_dict_service(full_type_name):
    [package_name, namespace, type_name] = full_type_name.split('/')
    package = importlib.import_module(f"{package_name}.srv")
    attributes = getattr(package, type_name)
    request_attributes = attributes.Request.get_fields_and_field_types()
    response_attributes = attributes.Response.get_fields_and_field_types()
    
    result = {
        full_type_name: {
            'request': process_attributes(request_attributes, namespace),
            'response': process_attributes(response_attributes, namespace)
        }
    }
    return result, set(list(request_attributes.values()) + list(response_attributes.values()))

def generate_attribute_dict_message(full_type_name):
    [package_name, namespace, type_name] = full_type_name.split('/')
    package = importlib.import_module(f"{package_name}.msg")
    attributes = getattr(package, type_name).get_fields_and_field_types()
    
    return {full_type_name: process_attributes(attributes, namespace)}, list(attributes.values())

def process_attributes(attributes, namespace):
    result = {}
    for attribute, attribute_type in attributes.items():
        if '/' in attribute_type:
            package_name, msg_name = attribute_type.split('/')
            result[attribute] = f"{package_name}/{namespace}/{msg_name}"
        else:
            result[attribute] = attribute_type
    return result

# def ROS2_to_vtkObject(full_type_name, output_directory, is_service=False):
#     [package, namespace, msg_name] = full_type_name.split('/')
#     print(f"Generating code for message: {full_type_name}")

#     vtk_class_name, vtk_type_identifier = get_class_name_formatted(full_type_name)
#     if vtk_class_name in vtk_equivalent_types.keys():
#         return
    
#     # is_service = '/srv/' in full_type_name

#     hpp_code = ""
#     cpp_code = ""
#     class_definitions_code_hpp = ""
#     class_definitions_code_cpp = ""

#     filename = f"vtk{vtk_class_name}"
#     hpp_code += f"#ifndef {filename}_h\n"
#     hpp_code += f"#define {filename}_h\n\n"

#     cpp_code += f"#include \"{filename}.h\"\n\n"

#     # message_attribute_map, unique_attributes = generate_attribute_dict(full_type_name, is_service)
#     if is_service:
#         message_attribute_map, unique_attributes = generate_attribute_dict_service(full_type_name)
#     else:
#         message_attribute_map, unique_attributes = generate_attribute_dict_message(full_type_name)

#     imports = identify_imports(msg_name, namespace, package, unique_attributes)
#     hpp_code += imports

#     generation_class_stack = []

#     if is_service:
#         for io_variable in ['request', 'response']:
#             generation_class_stack.append((vtk_class_name + io_variable.capitalize(), vtk_type_identifier + io_variable.capitalize(), message_attribute_map[full_type_name][io_variable], f"{package}::{namespace}::{msg_name}::{io_variable.capitalize()}"))
#     else:
#         generation_class_stack.append((vtk_class_name, vtk_type_identifier, message_attribute_map[full_type_name], f"{package}::{namespace}::{msg_name}"))

#     for class_name_formatted, class_type_identifier, fields, msg_ros2_type in generation_class_stack:

#         class_code_hpp_single, class_code_cpp_single = generate_class(class_name_formatted, fields)
#         hpp_code += "\n"
#         hpp_code += class_code_hpp_single
#         cpp_code += class_code_cpp_single

#         vtk_equivalent_types[class_type_identifier] = class_name_formatted

#         # hpp_code += "\n"
#         # hpp_code += class_definitions_code_hpp
#         # cpp_code += class_definitions_code_cpp

#         # Add Slicer to ROS2 conversion functions and vice versa
#         hpp_code += f"// Conversion functions\n"

        
#         hpp_code_single, cpp_code_single = generate_slicer_to_ros2_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields)
#         hpp_code += hpp_code_single
#         cpp_code += cpp_code_single

#         hpp_code_single, cpp_code_single = generate_ros2_to_slicer_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields)
#         hpp_code += hpp_code_single
#         cpp_code += cpp_code_single


#     hpp_code += f"\n#endif // {filename}_h\n"


#     with open(output_directory + '/' + filename + '.h', 'w') as h:
#         h.write(hpp_code)

#     with open(output_directory + '/' + filename + '.cxx', 'w') as cxx:
#         cxx.write(cpp_code)


def write_files(output_directory, filename, hpp_code, cpp_code):
    with open(output_directory + '/' + filename + '.h', 'w') as h:
        h.write(hpp_code)

    with open(output_directory + '/' + filename + '.cxx', 'w') as cxx:
        cxx.write(cpp_code)

def generate_class_and_conversion_methods(class_name_formatted, class_type_identifier, fields, msg_ros2_type):
    hpp_code, cpp_code = "\n", "\n"
    class_code_hpp_single, class_code_cpp_single = generate_class(class_name_formatted, fields)
    hpp_code += class_code_hpp_single
    cpp_code += class_code_cpp_single

    vtk_equivalent_types[class_type_identifier] = class_name_formatted

    # Add Slicer to ROS2 conversion functions and vice versa
    hpp_code += f"// Conversion functions\n"
    hpp_code_single, cpp_code_single = generate_slicer_to_ros2_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields)
    hpp_code += hpp_code_single
    cpp_code += cpp_code_single
    hpp_code_single, cpp_code_single = generate_ros2_to_slicer_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields)
    hpp_code += hpp_code_single
    cpp_code += cpp_code_single

    return hpp_code, cpp_code

def ROS2_service_to_vtkObject(full_type_name, output_directory):
    [package, namespace, msg_name] = full_type_name.split('/')
    print(f"Generating code for message: {full_type_name}")

    vtk_class_name, vtk_type_identifier = get_class_name_formatted(full_type_name)
    if vtk_class_name in vtk_equivalent_types.keys():
        return

    message_attribute_map, unique_attributes = generate_attribute_dict_service(full_type_name)

    filename = f"vtk{vtk_class_name}"
    hpp_code = f"#ifndef {filename}_h\n"
    hpp_code += f"#define {filename}_h\n\n"
    cpp_code = f"#include \"{filename}.h\"\n\n"

    imports = identify_imports(msg_name, namespace, package, unique_attributes)
    hpp_code += imports

    for io_variable in ['request', 'response']:
        class_name_formatted = vtk_class_name + io_variable.capitalize()
        class_type_identifier = vtk_type_identifier + io_variable.capitalize()
        fields = message_attribute_map[full_type_name][io_variable]
        msg_ros2_type = f"{package}::{namespace}::{msg_name}::{io_variable.capitalize()}"

        hpp_class_and_conversion_methods, cpp_class_and_conversion_methods = generate_class_and_conversion_methods(class_name_formatted, class_type_identifier, fields, msg_ros2_type)
        hpp_code += hpp_class_and_conversion_methods
        cpp_code += cpp_class_and_conversion_methods

    hpp_code += f"\n#endif // {filename}_h\n"

    write_files(output_directory, filename, hpp_code, cpp_code)


def ROS2_message_to_vtkObject(full_type_name, output_directory):
    [package, namespace, msg_name] = full_type_name.split('/')
    print(f"Generating code for message: {full_type_name}")

    vtk_class_name, vtk_type_identifier = get_class_name_formatted(full_type_name)
    if vtk_class_name in vtk_equivalent_types.keys():
        return

    filename = f"vtk{vtk_class_name}"
    hpp_code = f"#ifndef {filename}_h\n"
    hpp_code += f"#define {filename}_h\n\n"
    cpp_code = f"#include \"{filename}.h\"\n\n"

    message_attribute_map, unique_attributes = generate_attribute_dict_message(full_type_name)

    imports = identify_imports(msg_name, namespace, package, unique_attributes)
    hpp_code += imports

    hpp_class_and_conversion_methods, cpp_class_and_conversion_methods = generate_class_and_conversion_methods(vtk_class_name, vtk_type_identifier, message_attribute_map[full_type_name], f"{package}::{namespace}::{msg_name}")
    hpp_code += hpp_class_and_conversion_methods
    cpp_code += cpp_class_and_conversion_methods
    hpp_code += f"\n#endif // {filename}_h\n"

    write_files(output_directory, filename, hpp_code, cpp_code)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--message', type=str,
                        help='ROS message type. For example "geometry_msgs/msg/PoseStamped"')
    # parser.add_argument('-s', '--service', type=str,
    #                     help='ROS service type. For example "turtlesim/srv/Spawn"')
    parser.add_argument('-c', '--class-name', type=str, required=True)
    parser.add_argument('-d', '--directory', type=str, required=True)
    args = parser.parse_args()
    # ROS2_to_vtkObject(args.message, args.directory)

    is_service = '/srv/' in args.message

    if not is_service:
        ROS2_message_to_vtkObject(args.message, args.directory)
    else:
        ROS2_service_to_vtkObject(args.message, args.directory)

