#!/usr/bin/python3
"""
This version uses dictionaries of lambda functions (one per “category” of attribute)
so that we avoid having a long chain of if/else statements. The four categories
are:
   (is_sequence, is_vtk_object) = 
     FieldCategory.STATIC  → static (primitive) type
     FieldCategory.VTK_OBJECT   → a VTK object
     FieldCategory.STATIC_SEQUENCE   → a static sequence (std::vector of primitives)
     FieldCategory.VTK_SEQUENCE    → a VTK sequence (std::vector of vtkSmartPointer)
All code-generation (get/set methods, attribute declarations, initialization,
printing, and conversion functions) is done via a lookup in a mapping.
"""

import argparse
import importlib
from configForCodegen import ros2_to_cpp_type_static_mapping, static_cpp_type_default_value, vtk_equivalent_types, vtk_ignored_types
from utils import snake_to_camel, camel_to_snake, is_vtk_object, get_vtk_type
from enum import Enum, auto

# Global indent strings for formatting generated code
__   = "  "      # one-space indent
____ = "    "    # three-space indent


class FieldCategory(Enum):
    STATIC = auto()            ## TODO: Use Native instead
    VTK_OBJECT = auto() 
    STATIC_SEQUENCE = auto()    
    VTK_SEQUENCE = auto()      ## TODO: VTK_OBJECT_SEQUENCE

def get_category_mapping(is_sequence, is_vtk_object):
    """
    Returns the FieldCategory based on whether the attribute is a sequence and/or a VTK object.
    """
    CATEGORY_MAPPING = {
        (False, False): FieldCategory.STATIC,
        (False, True): FieldCategory.VTK_OBJECT,
        (True, False): FieldCategory.STATIC_SEQUENCE,
        (True, True): FieldCategory.VTK_SEQUENCE
    }

    return CATEGORY_MAPPING[(is_sequence, is_vtk_object)]

# ---------------------------------------------------------------------
# GET/SET method generators

GETSET_GENERATORS = {
    FieldCategory.STATIC: lambda attribute_name, underlying_type: (
        f"{__}// static get/set\n"
        f"{__}const {ros2_to_cpp_type_static_mapping[underlying_type]}& Get{snake_to_camel(attribute_name)}() const {{ return {attribute_name}_; }}\n\n"
        f"{__}void Set{snake_to_camel(attribute_name)}(const {ros2_to_cpp_type_static_mapping[underlying_type]}& value) {{ {attribute_name}_ = value; }}\n\n"
    ),
    FieldCategory.VTK_OBJECT: lambda attribute_name, underlying_type: (
        f"{__}// vtk get/set\n"
        f"{__}vtk{get_vtk_type(underlying_type, vtk_equivalent_types)[1]}* Get{snake_to_camel(attribute_name)}() {{ return {attribute_name}_; }}\n\n"
        f"{__}void Set{snake_to_camel(attribute_name)}(vtk{get_vtk_type(underlying_type, vtk_equivalent_types)[1]}* value) {{ {attribute_name}_ = value; }}\n\n"
    ),
    FieldCategory.STATIC_SEQUENCE: lambda attribute_name, underlying_type: (
        f"{__}// static sequence get/set\n"
        f"{__}const std::vector<{ros2_to_cpp_type_static_mapping[underlying_type]}>& Get{snake_to_camel(attribute_name)}() const {{ return {attribute_name}_; }}\n\n"
        f"{__}void Set{snake_to_camel(attribute_name)}(const std::vector<{ros2_to_cpp_type_static_mapping[underlying_type]}>& value) {{ {attribute_name}_ = value; }}\n\n"
    ),
    FieldCategory.VTK_SEQUENCE: lambda attribute_name, underlying_type: (
        f"{__}// vtk sequence get/set\n"
        f"{__}const std::vector<vtkSmartPointer<vtk{get_vtk_type(underlying_type, vtk_equivalent_types)[1]}>>& Get{snake_to_camel(attribute_name)}() const {{ return {attribute_name}_; }}\n\n"
        f"{__}void Set{snake_to_camel(attribute_name)}(const std::vector<vtkSmartPointer<vtk{get_vtk_type(underlying_type, vtk_equivalent_types)[1]}>>& value) {{ {attribute_name}_ = value; }}\n\n"
    )
}

# ---------------------------------------------------------------------
# Attribute declarations for the header file

ATTRIBUTE_DECLARATIONS = {
    FieldCategory.STATIC: lambda attribute_name, underlying_type: f"{__}{ros2_to_cpp_type_static_mapping[underlying_type]} {attribute_name}_;\n",
    FieldCategory.VTK_OBJECT: lambda attribute_name, underlying_type: f"{__}vtkSmartPointer<vtk{get_vtk_type(underlying_type, vtk_equivalent_types)[1]}> {attribute_name}_;\n",
    FieldCategory.STATIC_SEQUENCE: lambda attribute_name, underlying_type: f"{__}std::vector<{ros2_to_cpp_type_static_mapping[underlying_type]}> {attribute_name}_;\n",
    FieldCategory.VTK_SEQUENCE: lambda attribute_name, underlying_type: f"{__}std::vector<vtkSmartPointer<vtk{get_vtk_type(underlying_type, vtk_equivalent_types)[1]}>> {attribute_name}_;\n"
}

# ---------------------------------------------------------------------
# Attribute initialization code for the .cpp file

ATTRIBUTE_INIT = {
    FieldCategory.STATIC: lambda attribute_name, underlying_type: f"{__}{attribute_name}_ = {static_cpp_type_default_value.get(underlying_type, '0')};\n",
    FieldCategory.VTK_OBJECT: lambda attribute_name, underlying_type: f"{__}{attribute_name}_ = vtk{get_vtk_type(underlying_type, vtk_equivalent_types)[1]}::New();\n",
    FieldCategory.STATIC_SEQUENCE: lambda attribute_name, underlying_type: "",  # Empty vector by default.
    FieldCategory.VTK_SEQUENCE: lambda attribute_name, underlying_type: f"{__}{attribute_name}_ = std::vector<vtkSmartPointer<vtk{get_vtk_type(underlying_type, vtk_equivalent_types)[1]}>>();\n"
}

# How to generate PrintSelf code for each attribute.
# (For sequences we also use a loop to iterate over each element.)
PRINT_FIELD_GENERATORS = {
    FieldCategory.STATIC: lambda attribute_name, underlying_type, is_fixed_size: f"{__}os << indent << \"{snake_to_camel(attribute_name)}:\" << {attribute_name}_ << std::endl;\n",
    FieldCategory.VTK_OBJECT: lambda attribute_name, underlying_type, is_fixed_size: (
        f"{__}os << indent << \"{snake_to_camel(attribute_name)}:\" << std::endl;\n"
        f"{__}{attribute_name}_->PrintSelf(os, indent.GetNextIndent());\n"
    ),
    FieldCategory.STATIC_SEQUENCE: lambda attribute_name, underlying_type, is_fixed_size: (
        f"{__}os << indent << \"{snake_to_camel(attribute_name)}:\";\n"
        f"{__}for (const auto & __data : {attribute_name}_) os << __data << \" \";\n"
        f"{__}os << std::endl;\n"
    ),
    FieldCategory.VTK_SEQUENCE: lambda attribute_name, underlying_type, is_fixed_size: (
        f"{__}os << indent << \"{snake_to_camel(attribute_name)}:\" << std::endl;\n"
        f"{__}for (const auto & __data : {attribute_name}_) {{\n"
        f"{____}__data->PrintSelf(os, indent.GetNextIndent());\n"
        f"{__}}}\n"
        f"{__}os << std::endl;\n"
    )
}

# Slicer → ROS2 conversion code for each attribute.
# (For sequences, we use a for-loop; note that if the sequence isn’t fixed-size, we include a resize.)
SLICER2ROS_GENERATORS = {
    FieldCategory.STATIC: lambda attribute_name, camel_case_attribute_name, is_fixed_size: f"{__}result.{attribute_name} = input->Get{camel_case_attribute_name}();\n",
    FieldCategory.VTK_OBJECT: lambda attribute_name, camel_case_attribute_name, is_fixed_size: f"{__}vtkSlicerToROS2(input->Get{camel_case_attribute_name}(), result.{attribute_name}, rosNode);\n",
    FieldCategory.STATIC_SEQUENCE: lambda attribute_name, camel_case_attribute_name, is_fixed_size: (
        (f"{__}result.{attribute_name}.resize(input->Get{camel_case_attribute_name}().size());\n" if not is_fixed_size else "") +
        f"{__}std::copy(input->Get{camel_case_attribute_name}().begin(), input->Get{camel_case_attribute_name}().end(), result.{attribute_name}.begin());\n"
    ),
    FieldCategory.VTK_SEQUENCE: lambda attribute_name, camel_case_attribute_name, is_fixed_size: (
        (f"{__}result.{attribute_name}.resize(input->Get{camel_case_attribute_name}().size());\n" if not is_fixed_size else "") +
        f"{__}for (size_t i = 0; i < input->Get{camel_case_attribute_name}().size(); ++i) {{\n"
        f"{____}vtkSlicerToROS2(input->Get{camel_case_attribute_name}()[i], result.{attribute_name}[i], rosNode);\n"
        f"{__}}}\n"
    )
}

# ROS2 → Slicer conversion code for each attribute.
ROS2SLICER_GENERATORS = {
    FieldCategory.STATIC: lambda attribute_name, camel_case_attribute_name, underlying_type: f"{__}result->Set{camel_case_attribute_name}(input.{attribute_name});\n",
    FieldCategory.VTK_OBJECT: lambda attribute_name, camel_case_attribute_name, underlying_type: (
        f"{__}vtkSmartPointer<vtk{get_vtk_type(underlying_type, vtk_equivalent_types)[1]}> {attribute_name} = vtkSmartPointer<vtk{get_vtk_type(underlying_type, vtk_equivalent_types)[1]}>::New();\n"
        f"{__}vtkROS2ToSlicer(input.{attribute_name}, {attribute_name});\n"
        f"{__}result->Set{camel_case_attribute_name}({attribute_name});\n"
    ),
    FieldCategory.STATIC_SEQUENCE: lambda attribute_name, camel_case_attribute_name, underlying_type: (
        f"{__}std::vector<{ros2_to_cpp_type_static_mapping[underlying_type]}> temp_{attribute_name};\n"
        f"{__}temp_{attribute_name}.resize(input.{attribute_name}.size());\n"
        f"{__}std::copy(input.{attribute_name}.begin(), input.{attribute_name}.end(), temp_{attribute_name}.begin());\n"
        f"{__}result->Set{camel_case_attribute_name}(temp_{attribute_name});\n"
    ),
    FieldCategory.VTK_SEQUENCE: lambda attribute_name, camel_case_attribute_name, underlying_type: (
        f"{__}std::vector<vtkSmartPointer<vtk{get_vtk_type(underlying_type, vtk_equivalent_types)[1]}>> temp_{attribute_name};\n"
        f"{__}temp_{attribute_name}.resize(input.{attribute_name}.size());\n"
        f"{__}for (size_t i = 0; i < input.{attribute_name}.size(); ++i) {{\n"
        f"{____}vtkSmartPointer<vtk{get_vtk_type(underlying_type, vtk_equivalent_types)[1]}> tmp = vtkSmartPointer<vtk{get_vtk_type(underlying_type, vtk_equivalent_types)[1]}>::New();\n"
        f"{____}vtkROS2ToSlicer(input.{attribute_name}[i], tmp);\n"
        f"{____}temp_{attribute_name}[i] = tmp;\n"
        f"{__}}}\n"
        f"{__}result->Set{camel_case_attribute_name}(temp_{attribute_name});\n"
    )
}


#############################################################################
# HELPER FUNCTIONS

def get_field_category(attribute_type):
    """
    Returns a tuple: (is_sequence, underlying_type, is_vtk, is_fixed_size)
    If the attribute is a sequence then is_fixed_size is set according to the brackets.
    """
    if "sequence" in attribute_type or ('[' in attribute_type and ']' in attribute_type):
        if "sequence" in attribute_type:
            underlying_type = attribute_type.split('<')[1].split('>')[0].split(',')[0]
            is_fixed = False
        else:
            underlying_type = attribute_type.split('[')[0]
            is_fixed = True
        return underlying_type, True, is_vtk_object(underlying_type), is_fixed
    else:
        return attribute_type, False, is_vtk_object(attribute_type), None

def process_attribute(attr_type):
    """
    Returns a tuple (key, underlying_type, is_fixed, is_sequence) for an attribute.
    This consolidates calls to get_field_category and get_category_mapping.
    """
    underlying_type, is_sequence,  is_vtk, is_fixed = get_field_category(attr_type)
    category_key = get_category_mapping(is_sequence, is_vtk)
    return category_key, underlying_type, is_vtk, is_sequence, is_fixed

def get_class_name_formatted(ros_type_str):
    """
    For an input such as 'geometry_msgs/msg/PoseStamped' return:
      - ros_package_prefixed_message_name, e.g. "GeometryMsgsPoseStamped"
      - ros_message_name, e.g. "PoseStamped"
    """
    ros_pkg, ros_namespace, ros_message_name = ros_type_str.split('/')
    ros_package_prefixed_message_name = snake_to_camel(ros_pkg) + ros_message_name
    return ros_package_prefixed_message_name, ros_message_name

#############################################################################
# CODE GENERATION FUNCTIONS

def identify_imports(ros_message_name, ros_namespace, ros_pkg, attribute_types_list):
    imports = "// identify_imports\n"
    imports += "#include <vtkObject.h>\n#include <vtkNew.h>\n#include <string>\n#include <vtkSmartPointer.h>\n#include <vtkMRMLNode.h>\n\n"
    imports += f"#include <rclcpp/rclcpp.hpp>\n#include <{ros_pkg}/{ros_namespace}/{camel_to_snake(ros_message_name)}.hpp>\n"
    imports += "#include <vtkROS2ToSlicer.h>\n#include <vtkSlicerToROS2.h>\n"

    for attr_type in attribute_types_list:
        _, underlying_type, is_vtk_flag, _, _ = process_attribute(attr_type)
        if is_vtk_flag:
            _, vtk_underlying = get_vtk_type(underlying_type, vtk_equivalent_types)
            imports += f"#include <vtk{vtk_underlying}.h>\n"
    return imports

def format_constant(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    elif isinstance(value, (int, float)):
        return str(value)
    elif isinstance(value, bytes):
        # convert to an integer literal (or if needed, a char literal)
        return str(int.from_bytes(value, byteorder='big'))
    elif isinstance(value, str):
        return f"\"{value}\""
    else:
        return str(value)

def generate_class(generated_class_name, attributes, default_mapping = {}):
    # Generate .cpp code header and constructor initialization
    class_code_cpp = "// generate_class\n"
    class_code_cpp += f"vtkStandardNewMacro(vtk{generated_class_name});\n\n"
    class_code_cpp += f"vtk{generated_class_name}::vtk{generated_class_name}()\n{{\n"

    constant_decl = ""
    for const_name, const_value in default_mapping.items():
        # If the constant name does NOT end with __DEFAULT, add a constant declaration.
        if not const_name.endswith("__DEFAULT"):
            # Use a helper to format the constant (type and literal conversion)
            constant_decl += f"{__}static constexpr auto {const_name} = {format_constant(const_value)};\n"
    
    # Generate .hpp header for the class
    class_code_hpp = "// generate_class\n"
    class_code_hpp += f"class vtk{generated_class_name} : public vtkObject\n{{\npublic:\n"
    class_code_hpp += constant_decl
    class_code_hpp += f"{__}vtkTypeMacro(vtk{generated_class_name}, vtkObject);\n"
    class_code_hpp += f"{__}static vtk{generated_class_name}* New(void);\n\n"
    class_code_hpp += f"{__}void PrintSelf(std::ostream& os, vtkIndent indent) override;\n\n"
    
    method_code_hpp = ""
    attribute_decl_code_hpp = ""
    attribute_init_code_cpp = ""
    
    # Iterate over each attribute and generate code by dictionary lookup.
    for attribute_name, attr_type in attributes.items():
        field_category, underlying_type, _, _, _ = process_attribute(attr_type)
        method_code_hpp += GETSET_GENERATORS[field_category](attribute_name, underlying_type)
        attribute_decl_code_hpp += ATTRIBUTE_DECLARATIONS[field_category](attribute_name, underlying_type)
        attribute_init_code_cpp += ATTRIBUTE_INIT[field_category](attribute_name, underlying_type)
    
    class_code_hpp += method_code_hpp
    class_code_hpp += "protected:\n" + attribute_decl_code_hpp
    class_code_cpp += attribute_init_code_cpp
    class_code_cpp += "}\n\n"
    class_code_cpp += f"vtk{generated_class_name}::~vtk{generated_class_name}() = default;\n\n"
    
    class_code_hpp += f"\n{__}vtk{generated_class_name}();\n"
    class_code_hpp += f"{__}~vtk{generated_class_name}() override;\n"
    class_code_hpp += "};\n\n"
    
    return class_code_hpp, class_code_cpp

def generate_print_self_methods_for_class(generated_class_name, generated_class_identifier, attributes):
    code = "\n// generate_print_self_methods_for_class\n"
    generated_class_name = vtk_equivalent_types.get(generated_class_identifier, generated_class_name)
    code += f"void vtk{generated_class_name}::PrintSelf(std::ostream& os, vtkIndent indent) {{\n"
    code += f"{__}Superclass::PrintSelf(os, indent);\n"

    for attribute_name, attr_type in attributes.items():
        field_category, underlying_type, _, _, is_fixed = process_attribute(attr_type)
        code += PRINT_FIELD_GENERATORS[field_category](attribute_name, underlying_type, is_fixed)
    code += "}\n\n"
    return code

def generate_slicer_to_ros2_methods_for_class(generated_class_name, ros2_message_type, attributes):
    hpp = "// generate_slicer_to_ros2_methods_for_class\n"
    hpp += f"void vtkSlicerToROS2(vtk{generated_class_name}* input, {ros2_message_type} & result, const std::shared_ptr<rclcpp::Node>& rosNode);\n"
    cpp = "\n// generate_slicer_to_ros2_methods_for_class\n"
    cpp += f"void vtkSlicerToROS2(vtk{generated_class_name}* input, {ros2_message_type} & result, const std::shared_ptr<rclcpp::Node>& rosNode) {{\n"
    cpp += f"{__}(void)rosNode; // Suppress unused parameter warning\n"

    for attribute_name, attr_type in attributes.items():
        field_category, _, _, _, is_fixed_size = process_attribute(attr_type)
        camel_case_attribute_name = snake_to_camel(attribute_name)
        cpp += SLICER2ROS_GENERATORS[field_category](attribute_name, camel_case_attribute_name, is_fixed_size)
    cpp += "}\n\n"
    return hpp, cpp

def generate_ros2_to_slicer_methods_for_class(generated_class_name, ros2_message_type, attributes):
    hpp = "// generate_ros2_to_slicer_methods_for_class\n"
    hpp += f"void vtkROS2ToSlicer(const {ros2_message_type}& input, vtkSmartPointer<vtk{generated_class_name}> result);\n"
    cpp = "\n// generate_ros2_to_slicer_methods_for_class\n"
    cpp += f"void vtkROS2ToSlicer(const {ros2_message_type}& input, vtkSmartPointer<vtk{generated_class_name}> result) {{\n"

    for attribute_name, attr_type in attributes.items():
        field_category, underlying_type, _, _, _ = process_attribute(attr_type)
        camel_case_attribute_name = snake_to_camel(attribute_name)
        cpp += ROS2SLICER_GENERATORS[field_category](attribute_name, camel_case_attribute_name, underlying_type)
    cpp += "}\n\n"
    return hpp, cpp


def generate_class_and_conversion_methods(generated_class_name, generated_class_identifier, attributes, ros2_message_type, constants = {}):
    hpp, cpp = "\n", "\n"
    hpp_class, cpp_class = generate_class(generated_class_name, attributes, constants)
    hpp += hpp_class
    cpp += cpp_class

    vtk_equivalent_types[generated_class_identifier] = generated_class_name

    cpp += generate_print_self_methods_for_class(generated_class_name, generated_class_identifier, attributes)
    hpp_conv, cpp_conv = generate_slicer_to_ros2_methods_for_class(generated_class_name, ros2_message_type, attributes)
    hpp += hpp_conv
    cpp += cpp_conv
    hpp_ros, cpp_ros = generate_ros2_to_slicer_methods_for_class(generated_class_name, ros2_message_type, attributes)
    hpp += hpp_ros
    cpp += cpp_ros

    return hpp, cpp

#############################################################################

def process_attributes(attributes, ros_namespace):
    result = {
        attr: f"{attr_type.split('/')[0]}/{ros_namespace}/{attr_type.split('/')[1]}" 
        if '/' in attr_type 
        else attr_type
        for attr, attr_type in attributes.items()
    }
    
    # Create a set of ignored types for O(1) lookup
    ignored_types_set = set(vtk_ignored_types.keys())
    
    # Filter out ignored types in a single pass
    return {
        attr: attr_type 
        for attr, attr_type in result.items()
        if not any(ignored in attr_type for ignored in ignored_types_set)
    }

def generate_attribute_dict_message(ros_message_full_type):
    ros_pkg, ros_namespace, ros_message_name = ros_message_full_type.split('/')
    mod = importlib.import_module(f"{ros_pkg}.msg")
    attributes = getattr(mod, ros_message_name).get_fields_and_field_types()
    msg_class = getattr(mod, ros_message_name)
    constants = {name: value for name, value in vars(msg_class).items() 
                 if name.isupper() and not name.startswith('_')}
    constants.pop('SLOT_TYPES', None)
    # print(f"constants: {constants}")
    processed_attributes = process_attributes(attributes, ros_namespace)
    return {ros_message_full_type: processed_attributes}, list(processed_attributes.values()), constants

def generate_attribute_dict_service(ros_service_full_type):
    ros_pkg, ros_namespace, ros_service_name = ros_service_full_type.split('/')
    mod = importlib.import_module(f"{ros_pkg}.srv")
    srv = getattr(mod, ros_service_name)
    request_attributes = srv.Request.get_fields_and_field_types()
    response_attributes = srv.Response.get_fields_and_field_types()
    processed_request_attributes = process_attributes(request_attributes, ros_namespace)
    processed_response_attributes = process_attributes(response_attributes, ros_namespace)
    unique_attribute_types = list(set(list(request_attributes.values()) + list(response_attributes.values())))

    return {ros_service_full_type: {'request': processed_request_attributes,
                                    'response': processed_response_attributes}}, unique_attribute_types

def write_files(output_directory, filename, hpp_code, cpp_code):
    with open(f"{output_directory}/{filename}.h", 'w') as h:
        h.write(hpp_code)
    with open(f"{output_directory}/{filename}.cxx", 'w') as cxx:
        cxx.write(cpp_code)


#############################################################################
# HIGH-LEVEL FUNCTIONS

def ROS2_message_to_vtkObject(ros_message_full_type, output_directory):
    ros_pkg, ros_namespace, ros_message_name = ros_message_full_type.split('/')
    print(f"Generating code for message: {ros_message_full_type}")
    generated_vtk_class_name, generated_vtk_class_identifier = get_class_name_formatted(ros_message_full_type)
    if generated_vtk_class_name in vtk_equivalent_types:
        return
    filename = f"vtk{generated_vtk_class_name}"
    hpp_code = f"#ifndef {filename}_h\n#define {filename}_h\n\n"
    cpp_code = f'#include "{filename}.h"\n\n#include <vtkROS2ToSlicer.h>\n#include <vtkSlicerToROS2.h>\n\n'
    message_attribute_map, unique_attribute_types, constants = generate_attribute_dict_message(ros_message_full_type)
    # print(f"unique_attributes: {unique_attribute_types}")
    # print(f"message_attribute_map: {message_attribute_map}")
    imports = identify_imports(ros_message_name, ros_namespace, ros_pkg, unique_attribute_types)
    hpp_code += imports
    hpp_conv, cpp_conv = generate_class_and_conversion_methods(generated_vtk_class_name, generated_vtk_class_identifier,
                                                               message_attribute_map[ros_message_full_type],
                                                               f"{ros_pkg}::{ros_namespace}::{ros_message_name}", constants)
    hpp_code += hpp_conv
    cpp_code += cpp_conv
    hpp_code += f"\n#endif // {filename}_h\n"
    write_files(output_directory, filename, hpp_code, cpp_code)

def ROS2_service_to_vtkObject(ros_service_full_type, output_directory): # TODO: Add constants
    ros_pkg, ros_namespace, ros_service_name = ros_service_full_type.split('/')
    print(f"Generating code for service: {ros_service_full_type}")
    generated_vtk_class_name, generated_vtk_class_identifier = get_class_name_formatted(ros_service_full_type)
    if generated_vtk_class_name in vtk_equivalent_types:
        return
    message_map, unique_attribute_types = generate_attribute_dict_service(ros_service_full_type)
    filename = f"vtk{generated_vtk_class_name}"
    hpp_code = f"""#ifndef {filename}_h\n#define {filename}_h\n\n"""
    cpp_code = f'#include "{filename}.h"\n\n#include <vtkROS2ToSlicer.h>\n#include <vtkSlicerToROS2.h>\n\n'
    imports = identify_imports(ros_service_name, ros_namespace, ros_pkg, unique_attribute_types)
    hpp_code += imports
    for io in ['request', 'response']:
        vtk_class_name_formatted = generated_vtk_class_name + io.capitalize()
        vtk_class_type_identifier = generated_vtk_class_identifier + io.capitalize()
        attributes = message_map[ros_service_full_type][io]
        ros2_message_type = f"{ros_pkg}::{ros_namespace}::{ros_service_name}::{io.capitalize()}"
        hpp_conv, cpp_conv = generate_class_and_conversion_methods(vtk_class_name_formatted, vtk_class_type_identifier,
                                                                   attributes, ros2_message_type)
        hpp_code += hpp_conv
        cpp_code += cpp_conv
    hpp_code += f"\n#endif // {filename}_h\n"
    write_files(output_directory, filename, hpp_code, cpp_code)

#############################################################################
# MAIN

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--message', type=str,
                        help='ROS message type. For example "geometry_msgs/msg/PoseStamped"')
    parser.add_argument('-s', '--service', type=str,
                        help='ROS service type. For example "turtlesim/srv/Spawn"')
    parser.add_argument('-c', '--class-name', type=str, required=True)
    parser.add_argument('-d', '--directory', type=str, required=True)
    args = parser.parse_args()
    if args.message:
        ROS2_message_to_vtkObject(args.message, args.directory)
    elif args.service:
        ROS2_service_to_vtkObject(args.service, args.directory)
