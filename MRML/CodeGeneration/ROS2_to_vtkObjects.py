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

def is_vector_needed(field_type):
    if "sequence" in field_type:
        return True
    # if something like [3] is present, it is a sequence
    if '[' in field_type and ']' in field_type:
        # print(f"field_type: {field_type}")
        return True
    return False

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


def get_type_from_sequence(sequence_type):
    #  FIXME: This is a hacky way to extract the type from the sequence type and also whether the size is fixed or not
    # fixed size sequences do not support resize, so we need to know if the size is fixed or not
    if "sequence" in sequence_type:
        # print(f"sequence_type: {sequence_type}")
        extracted = sequence_type.split('<')[1].split('>')[0].split(',')[0]
        # print(f"extracted: {extracted}")
        return extracted, False
    if '[' in sequence_type and ']' in sequence_type:
        extracted = sequence_type.split('[')[0]
        return extracted, True
    return _, _


def generate_vtk_getset(method_code_hpp, field_name, field_type):
    method_code_hpp += f"{__} // generate_vtk_getset\n"
    method_code_hpp += f"{__} vtk{field_type} * Get{snake_to_camel(field_name)}(void) {{\n"
    method_code_hpp += f"{____} return {field_name}_;\n"
    method_code_hpp += f"{__} }}\n\n"
    method_code_hpp += f"{__} void Set{snake_to_camel(field_name)}(vtk{field_type} * value) {{\n"
    method_code_hpp += f"{____} {field_name}_ = value;\n"
    method_code_hpp += f"{__} }}\n\n"

    return method_code_hpp


def generate_static_getset(method_code_hpp, field_name, field_type, static_type_mapping):
    method_code_hpp += f"{__}// generate_static_getset\n"
    method_code_hpp += f"{__} const {static_type_mapping[field_type]}& Get{snake_to_camel(field_name)}(void) const {{\n"
    method_code_hpp += f"{____} return {field_name}_;\n"
    method_code_hpp += f"{__} }}\n\n"
    method_code_hpp += f"{__} void Set{snake_to_camel(field_name)}(const {static_type_mapping[field_type]}& value) {{\n"
    method_code_hpp += f"{____} {field_name}_ = value;\n"
    method_code_hpp += f"{__} }}\n\n"

    return method_code_hpp


def generate_static_sequence_getset(method_code_hpp, field_name, field_type, static_type_mapping):
    method_code_hpp += f"    const std::vector<{static_type_mapping[field_type]}>& Get{snake_to_camel(field_name)}(void) const {{\n"
    method_code_hpp += f"        return {field_name}_;\n"
    method_code_hpp += f"    }}\n\n"

    method_code_hpp += f"    void Set{snake_to_camel(field_name)}(const std::vector<{static_type_mapping[field_type]}>& value) {{\n"
    method_code_hpp += f"        {field_name}_ = value;\n"
    method_code_hpp += f"    }}\n\n"

    return method_code_hpp


def generate_vtk_sequence_getset(method_code_hpp, field_name, field_type, static_type_mapping):
    method_code_hpp += f"    const std::vector<vtkSmartPointer<vtk{field_type}>>& Get{snake_to_camel(field_name)}(void) const {{\n"
    method_code_hpp += f"        return {field_name}_;\n"
    method_code_hpp += f"    }}\n\n"

    method_code_hpp += f"    void Set{snake_to_camel(field_name)}(const std::vector<vtkSmartPointer<vtk{field_type}>>& value) {{\n"
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
    class_code_hpp += f"{__} void PrintSelf(std::ostream& os, vtkIndent indent) override;\n\n"

    method_code_hpp = ""
    attribute_code_hpp = ""
    attribute_code_cpp = ""

    for field_name, field_type in fields.items():
        # Check if the field is a VTK object or not
        # if "sequence" in field_type:
        if is_vector_needed(field_type):
            element_type, is_fixed_size = get_type_from_sequence(field_type)
            # check if element_type is a vtk object or static type
            if is_vtk_object(element_type):
                is_equivalent_type_available, element_type = get_vtk_type(element_type, vtk_equivalent_types)
                method_code_hpp = generate_vtk_sequence_getset(method_code_hpp, field_name, element_type, static_type_mapping)
                attribute_code_hpp += f"{__} std::vector<vtkSmartPointer<vtk{element_type}>> {field_name}_;\n"
                attribute_code_cpp += f"{__} {field_name}_ = std::vector<vtkSmartPointer<vtk{element_type}>>();\n"
            else:
                method_code_hpp = generate_static_sequence_getset(method_code_hpp, field_name, element_type, static_type_mapping)
                attribute_code_hpp += f"{__} std::vector<{static_type_mapping[element_type]}> {field_name}_;\n"

        elif is_vtk_object(field_type):
            is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
            method_code_hpp = generate_vtk_getset(method_code_hpp, field_name, field_type)
            attribute_code_hpp += f"{__} vtkSmartPointer<vtk{field_type}> {field_name}_;\n"
            attribute_code_cpp += f"{__} {field_name}_ = vtk{field_type}::New();\n"
        else:
            method_code_hpp = generate_static_getset(method_code_hpp, field_name, field_type, static_type_mapping)
            attribute_code_hpp += f"{__} {static_type_mapping[field_type]} {field_name}_;\n"
            attribute_code_cpp += f"{__} {field_name}_ = {static_type_default_value.get(field_type, '0')};\n"

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
        # if "sequence" in field_type:
        if is_vector_needed(field_type):
            element_type, is_fixed_size = get_type_from_sequence(field_type)
            if is_vtk_object(element_type):
                is_equivalent_type_available, element_type = get_vtk_type(element_type, vtk_equivalent_types)
                imports += f"#include <vtk{element_type}.h>\n"
        elif is_vtk_object(field_type):
            is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
            imports += f"#include <vtk{field_type}.h>\n"
    return imports

def generate_print_self_methods_for_class(class_name_formatted, class_type_identifier, fields):
    code_string_cpp = "\n// generate_print_self_methods_for_class\n"

    class_name_formatted = vtk_equivalent_types.get(class_type_identifier, class_name_formatted)
    code_string_cpp += f"void vtk{class_name_formatted}::PrintSelf(std::ostream& os, vtkIndent indent) {{\n"

    code_string_cpp+= f"{__} Superclass::PrintSelf(os, indent);\n";

    for field_name, field_type in fields.items():

        # if "sequence" in field_type:
        if is_vector_needed(field_type):
            element_type, is_fixed_size = get_type_from_sequence(field_type)
            # based on whether the element type is a vtk object or not, print the sequence. For vtk objects, print using calls to PrintSelf of the individual elements
            if is_vtk_object(element_type):
                is_equivalent_type_available, element_type = get_vtk_type(element_type, vtk_equivalent_types)
                code_string_cpp += f"{__} os << indent << \"{snake_to_camel(field_name)}:\" << std::endl;\n"
                code_string_cpp += f"{__} for (const auto & __data : {field_name}_) {{\n"
                code_string_cpp += f"{____} __data->PrintSelf(os, indent.GetNextIndent());\n"
                code_string_cpp += f"{__} }}\n"
                code_string_cpp += f"{__} os << std::endl;\n"
            else:
                code_string_cpp += f"{__} os << indent << \"{snake_to_camel(field_name)}:\";\n"
                code_string_cpp += f"{__} for (const auto & __data : {field_name}_) os << __data << \" \";\n"
                code_string_cpp += f"{__} os << std::endl;\n"

        elif is_vtk_object(field_type):
            code_string_cpp += f"{__} os << indent << \"{snake_to_camel(field_name)}:\" << std::endl;\n"
            code_string_cpp += f"{__} {field_name}_->PrintSelf(os, indent.GetNextIndent());\n"

        else:
            code_string_cpp += f"{__} os << indent << \"{snake_to_camel(field_name)}:\" << {field_name}_ << std::endl;\n"

    code_string_cpp += "}\n\n"
    return code_string_cpp

def generate_slicer_to_ros2_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields):
    code_string_cpp = "\n// generate_slicer_to_ros2_methods_for_class\n"
    code_string_hpp = "// generate_slicer_to_ros2_methods_for_class\n"

    class_name_formatted = vtk_equivalent_types.get(class_type_identifier, class_name_formatted)
    code_string_cpp += f"void vtkSlicerToROS2( vtk{class_name_formatted} * input, {msg_ros2_type} & result, const std::shared_ptr<rclcpp::Node> & rosNode) {{\n"
    code_string_hpp += f"void vtkSlicerToROS2( vtk{class_name_formatted} * input, {msg_ros2_type} & result, const std::shared_ptr<rclcpp::Node> & rosNode);\n"

    for field_name, field_type in fields.items():
        # if "sequence" in field_type:
        if is_vector_needed(field_type):
            element_type, is_fixed_size = get_type_from_sequence(field_type)
            if is_vtk_object(field_type):
                is_equivalent_type_available, vtk_field_type = get_vtk_type(field_type, vtk_equivalent_types)
                if not is_fixed_size:
                    code_string_cpp += f"{__} result.{field_name}.resize(input->Get{snake_to_camel(field_name)}().size());\n"
                code_string_cpp += f"{__} for (size_t i = 0; i < input->Get{snake_to_camel(field_name)}().size(); ++i) {{\n"
                code_string_cpp += f"{____} vtkSlicerToROS2(input->Get{snake_to_camel(field_name)}()[i], result.{field_name}[i], rosNode);\n"
                code_string_cpp += f"{__} }}\n"
            else:
                if not is_fixed_size:
                    code_string_cpp += f"{__} result.{field_name}.resize(input->Get{snake_to_camel(field_name)}().size());\n"
                code_string_cpp += f"{__} std::copy(input->Get{snake_to_camel(field_name)}().begin(), input->Get{snake_to_camel(field_name)}().end(), result.{field_name}.begin());\n"
        elif is_vtk_object(field_type):
            is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
            code_string_cpp += f"{__} vtkSlicerToROS2(input->Get{snake_to_camel(field_name)}(), result.{field_name}, rosNode);\n"
        else:
            code_string_cpp += f"{__} result.{field_name} = input->Get{snake_to_camel(field_name)}();\n"

    code_string_cpp += "}\n\n"
    return code_string_hpp, code_string_cpp


def generate_ros2_to_slicer_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields):
    code_string_cpp = "\n// generate_ros2_to_slicer_methods_for_class\n"
    code_string_hpp = "// generate_ros2_to_slicer_methods_for_class\n"
    class_name_formatted = vtk_equivalent_types.get(class_type_identifier, class_name_formatted)
    code_string_hpp += f"void vtkROS2ToSlicer(const {msg_ros2_type} & input, vtkSmartPointer<vtk{class_name_formatted}> result);\n"
    code_string_cpp += f"void vtkROS2ToSlicer(const {msg_ros2_type} & input, vtkSmartPointer<vtk{class_name_formatted}> result) {{\n"

    for field_name, field_type in fields.items():
        # if "sequence" in field_type:
        if is_vector_needed(field_type):
            element_type, is_fixed_size = get_type_from_sequence(field_type)
            if is_vtk_object(element_type):
                is_equivalent_type_available, vtk_element_type = get_vtk_type(element_type, vtk_equivalent_types)
                code_string_cpp += f"{__} std::vector<vtkSmartPointer<vtk{vtk_element_type}>> temp_{field_name};\n"
                code_string_cpp += f"{__} temp_{field_name}.resize(input.{field_name}.size());\n"
                code_string_cpp += f"{__} for (size_t i = 0; i < input.{field_name}.size(); ++i) {{\n"
                code_string_cpp += f"{____} vtkSmartPointer<vtk{vtk_element_type}> temp_{field_name}_element = vtkSmartPointer<vtk{vtk_element_type}>::New();\n"
                code_string_cpp += f"{____} vtkROS2ToSlicer(input.{field_name}[i], temp_{field_name}_element);\n"
                code_string_cpp += f"{____} temp_{field_name}[i] = temp_{field_name}_element;\n"
                code_string_cpp += f"{__} }}\n"
            else:
                static_element_type = static_type_mapping[element_type]
                code_string_cpp += f"{__} std::vector<{static_element_type}> temp_{field_name};\n"
                code_string_cpp += f"{__} temp_{field_name}.resize(input.{field_name}.size());\n"
                code_string_cpp += f"{__} std::copy(input.{field_name}.begin(), input.{field_name}.end(), temp_{field_name}.begin());\n"
            code_string_cpp += f"{__} result->Set{snake_to_camel(field_name)}(temp_{field_name});\n"
        elif is_vtk_object(field_type):
            is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
            code_string_cpp += f"{__} vtkSmartPointer<vtk{field_type}> {field_name} = vtkSmartPointer<vtk{field_type}>::New();\n"
            code_string_cpp += f"{__} vtkROS2ToSlicer(input.{field_name}, {field_name});\n"
            code_string_cpp += f"{__} result->Set{snake_to_camel(field_name)}({field_name});\n"
        else:
            code_string_cpp += f"{__} result->Set{snake_to_camel(field_name)}(input.{field_name});\n"

    code_string_cpp += "}\n\n"
    return code_string_hpp, code_string_cpp


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

    cpp_code += generate_print_self_methods_for_class(class_name_formatted, class_type_identifier, fields)

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
    print(f"Generating code for service: {full_type_name}")

    vtk_class_name, vtk_type_identifier = get_class_name_formatted(full_type_name)
    if vtk_class_name in vtk_equivalent_types.keys():
        return

    message_attribute_map, unique_attributes = generate_attribute_dict_service(full_type_name)

    filename = f"vtk{vtk_class_name}"
    hpp_code = f"#ifndef {filename}_h\n"
    hpp_code += f"#define {filename}_h\n\n"
    cpp_code = f"#include \"{filename}.h\"\n\n"
    cpp_code += f"#include <vtkROS2ToSlicer.h>\n"
    cpp_code += f"#include <vtkSlicerToROS2.h>\n\n"

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
    cpp_code += f"#include <vtkROS2ToSlicer.h>\n"
    cpp_code += f"#include <vtkSlicerToROS2.h>\n\n"

    message_attribute_map, unique_attributes = generate_attribute_dict_message(full_type_name)

    print(f"Type: {full_type_name}")

    print(f"unique_attributes : {unique_attributes}")

    print(f"message_attribute_map: {message_attribute_map}")

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
    parser.add_argument('-s', '--service', type=str,
                        help='ROS service type. For example "turtlesim/srv/Spawn"')
    parser.add_argument('-c', '--class-name', type=str, required=True)
    parser.add_argument('-d', '--directory', type=str, required=True)
    args = parser.parse_args()
    # ROS2_to_vtkObject(args.message, args.directory)

    if args.message:
        ROS2_message_to_vtkObject(args.message, args.directory)
    elif args.service:
        ROS2_service_to_vtkObject(args.service, args.directory)
