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
    method_code_hpp += f"    const std::vector<{static_type_mapping[field_type]}>& Get{field_name.capitalize()}(void) const {{\n"
    method_code_hpp += f"        return {field_name}_;\n"
    method_code_hpp += f"    }}\n\n"

    method_code_hpp += f"    void Set{field_name.capitalize()}(const std::vector<{static_type_mapping[field_type]}>& value) {{\n"
    method_code_hpp += f"        {field_name}_ = value;\n"
    method_code_hpp += f"    }}\n\n"

    return method_code_hpp


def generate_class(class_name, fields, message_attribute_map):

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
        if is_vtk_object(field_type, message_attribute_map):
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

def identify_imports(class_name, namespace, package_name, fields, message_attribute_map):
    imports = "// identify_imports\n"
    # usual suspects
    imports += "#include <vtkObject.h>\n"
    imports += "#include <vtkNew.h>\n"
    imports += "#include <string>\n"
    imports += "#include <vtkSmartPointer.h>\n"

    # include the message type
    imports += f"#include <rclcpp/rclcpp.hpp>\n"
    imports += f"#include <{package_name}/{namespace}/{camel_to_snake(class_name)}.hpp>\n"

    for field_name, field_type in fields.items():
        if is_vtk_object(field_type, message_attribute_map):
            is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
            imports += f"#include <vtk{field_type}.h>\n"

    return imports


def generate_print_self_methods_for_class(class_name_formatted, class_type_identifier, fields, message_attribute_map):
    code_string_cpp = "\n// generate_print_self_methods_for_class\n"

    class_name_formatted = vtk_equivalent_types.get(class_type_identifier, class_name_formatted)
    code_string_cpp += f"void vtk{class_name_formatted}::PrintSelf(std::ostream& os, vtkIndent indent) {{\n"

    code_string_cpp+= f"{__} Superclass::PrintSelf(os, indent);\n";

    for field_name, field_type in fields.items():
        if is_vtk_object(field_type, message_attribute_map):
            code_string_cpp += f"{__} os << indent << \"{field_name.capitalize()}:\" << std::endl;\n"
            code_string_cpp += f"{__} {field_name}_->PrintSelf(os, indent.GetNextIndent());\n"
        elif "sequence" in field_type:
            code_string_cpp += f"{__} os << indent << \"{field_name.capitalize()}:\";\n"
            code_string_cpp += f"{__} for (const auto & __data : {field_name}_) os << __data << \" \";\n"
            code_string_cpp += f"{__} os << std::endl;\n"

        else:
            code_string_cpp += f"{__} os << indent << \"{field_name.capitalize()}:\" << {field_name}_ << std::endl;\n"

    code_string_cpp += "}\n\n"
    return code_string_cpp


def generate_slicer_to_ros2_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields, message_attribute_map):
    code_string_cpp = "\n// generate_slicer_to_ros2_methods_for_class\n"
    code_string_hpp = "// generate_slicer_to_ros2_methods_for_class\n"

    class_name_formatted = vtk_equivalent_types.get(class_type_identifier, class_name_formatted)
    code_string_cpp += f"void vtkSlicerToROS2( vtk{class_name_formatted} * input, {msg_ros2_type} & result, const std::shared_ptr<rclcpp::Node> & rosNode) {{\n"
    code_string_hpp += f"void vtkSlicerToROS2( vtk{class_name_formatted} * input, {msg_ros2_type} & result, const std::shared_ptr<rclcpp::Node> & rosNode);\n"

    for field_name, field_type in fields.items():
        if is_vtk_object(field_type, message_attribute_map):
            is_equivalent_type_available, field_type = get_vtk_type(field_type, vtk_equivalent_types)
            code_string_cpp += f"{__} vtkSlicerToROS2(input->Get{field_name.capitalize()}(), result.{field_name}, rosNode);\n"
        elif "sequence" in field_type:
            code_string_cpp += f"{__} result.{field_name}.resize(input->Get{field_name.capitalize()}().size());\n"
            code_string_cpp += f"{__} std::copy(input->Get{field_name.capitalize()}().begin(), input->Get{field_name.capitalize()}().end(), result.{field_name}.begin());\n"
        else:
            code_string_cpp += f"{__} result.{field_name} = input->Get{field_name.capitalize()}();\n"

    code_string_cpp += "}\n\n"
    return code_string_hpp, code_string_cpp


def generate_ros2_to_slicer_methods_for_class(class_name_formatted, class_type_identifier, msg_ros2_type, fields, message_attribute_map):
    code_string_cpp = "\n// generate_ros2_to_slicer_methods_for_class\n"
    code_string_hpp = "// generate_ros2_to_slicer_methods_for_class\n"
    class_name_formatted = vtk_equivalent_types.get(class_type_identifier, class_name_formatted)
    code_string_hpp += f"void vtkROS2ToSlicer(const {msg_ros2_type} & input, vtkSmartPointer<vtk{class_name_formatted}> result);\n"
    code_string_cpp += f"void vtkROS2ToSlicer(const {msg_ros2_type} & input, vtkSmartPointer<vtk{class_name_formatted}> result) {{\n"

    for field_name, field_type in fields.items():
        if is_vtk_object(field_type, message_attribute_map):
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
    print(f"ROS2_to_vtkObject: generating code for message \"{_message}\"")

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

    filename = f"vtk{class_name_formatted}"
    hpp_code += f"#ifndef {filename}_h\n"
    hpp_code += f"#define {filename}_h\n\n"

    cpp_code += f"#include \"{filename}.h\"\n"
    cpp_code += f"#include <vtkROS2ToSlicer.h>\n"
    cpp_code += f"#include <vtkSlicerToROS2.h>\n\n"

    imports = identify_imports(msg_name, namespace, package, fields, message_attribute_map,)
    hpp_code += imports

    class_code_hpp_single, class_code_cpp_single = generate_class(class_name_formatted, fields, message_attribute_map)
    class_definitions_code_hpp += class_code_hpp_single
    class_definitions_code_cpp += class_code_cpp_single

    vtk_equivalent_types[class_type_identifier] = class_name_formatted

    hpp_code += "\n"
    hpp_code += class_definitions_code_hpp
    cpp_code += class_definitions_code_cpp

    cpp_code += generate_print_self_methods_for_class(class_name_formatted, class_type_identifier, fields, message_attribute_map)
    
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
