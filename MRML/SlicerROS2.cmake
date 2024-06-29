#
# generate ROS2 nodes as well as messages required
#
function(generate_ros2_nodes)
  set(_options)
  set(_oneValueArgs GENERATED_FILES)
  set(_multiValueArgs PUBLISHERS SUBSCRIBERS DEPENDENCIES)
  cmake_parse_arguments(_slicer_ros "${_options}" "${_oneValueArgs}" "${_multiValueArgs}" ${ARGN})

  set(_ros_messages)
  foreach(_msg ${_slicer_ros_PUBLISHERS} ${_slicer_ros_SUBSCRIBERS} ${_slicer_ros_DEPENDENCIES})
    list(APPEND _ros_messages ${_msg})
  endforeach()
  list(REMOVE_DUPLICATES _ros_messages)

  set(_all_files_generated)
  foreach(_msg ${_ros_messages})
    generate_ros2_message(${_msg} _files_generated)
    list(APPEND _all_files_generated ${_files_generated})
  endforeach ()

  set(h_file "${CMAKE_CURRENT_BINARY_DIR}/vtkMRMLROS2GeneratedNodes.h")
  set(cxx_file "${CMAKE_CURRENT_BINARY_DIR}/vtkMRMLROS2GeneratedNodes.cxx")

  set(_h "// automatically generated by CMake, do not edit manually\n")
  string(APPEND _h "\n#ifndef __vtkMRMLROS2GeneratedNodes_h\n#define __vtkMRMLROS2GeneratedNodes_h\n")
  string(APPEND _h "\n#include <vtkMRMLScene.h>\n")
  string(APPEND _h "\n#include <vtkMRMLROS2PublisherNode.h>\n#include <vtkMRMLROS2PublisherMacros.h>\n")
  string(APPEND _h "\n#include <vtkMRMLROS2SubscriberNode.h>\n#include <vtkMRMLROS2SubscriberMacros.h>\n")

  string(APPEND _h "\nvoid vtkMRMLROS2GeneratedNodesRegister(vtkSmartPointer<vtkMRMLScene>);\n")

  set(_cxx "// automatically generated by CMake, do not edit manually\n")
  string(APPEND _cxx "\n#include <vtkMRMLROS2GeneratedNodes.h>")
  string(APPEND _cxx "\n#include <vtkSlicerToROS2.h>\n#include <vtkMRMLROS2PublisherInternals.h>\n")
  string(APPEND _cxx "\n#include <vtkROS2ToSlicer.h>\n#include <vtkMRMLROS2SubscriberInternals.h>\n")

  set(_reg)
  foreach(_pub ${_slicer_ros_PUBLISHERS})
    generate_ros2_publisher_code(${_pub} _pub_h _pub_cxx _pub_reg)
    string(APPEND _h   "\n${_pub_h}")
    string(APPEND _cxx "\n${_pub_cxx}")
    string(APPEND _reg "\n${_pub_reg}")
  endforeach()

  foreach(_sub ${_slicer_ros_SUBSCRIBERS})
    generate_ros2_subscriber_code(${_sub} _sub_h _sub_cxx _sub_reg)
    string(APPEND _h   "\n${_sub_h}")
    string(APPEND _cxx "\n${_sub_cxx}")
    string(APPEND _reg "\n${_sub_reg}")
  endforeach()

  string(APPEND _h "\n#endif\n")

  string(APPEND _cxx "\nvoid vtkMRMLROS2GeneratedNodesRegister(vtkSmartPointer<vtkMRMLScene> scene) {")
  string(APPEND _cxx "${_reg}\n}\n")

  file(WRITE ${h_file} "${_h}")
  file(WRITE ${cxx_file} "${_cxx}")

  set(${_slicer_ros_GENERATED_FILES} ${_all_files_generated} ${h_file} ${cxx_file} PARENT_SCOPE)
endfunction()


#
# generate the vtk class corresponding to the ROS message as well as
# conversion methods between ROS and vtk
#
function(generate_ros2_message _msg _files_generated)
  # convert msg to class name
  ros2_msg_to_vtk_class(${_msg} _class_name)
  # create custom command
  set(_h_file "${CMAKE_CURRENT_BINARY_DIR}/${_class_name}.h")
  set(_cxx_file "${CMAKE_CURRENT_BINARY_DIR}/${_class_name}.cxx")
  set_source_files_properties(${_h} PROPERTIES GENERATED 1)
  set_source_files_properties(${_cxx} PROPERTIES GENERATED 1)
  set(_generator "${CMAKE_CURRENT_SOURCE_DIR}/CodeGeneration/ROS2_to_vtkObjects.py")
  set(_generator_dependencies
    "${CMAKE_CURRENT_SOURCE_DIR}/CodeGeneration/configForCodegen.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/CodeGeneration/utils.py")
  add_custom_command(
    OUTPUT ${_h_file} ${_cxx_file}
    COMMAND ${_generator}
    -m ${_msg}
    -c ${_class_name}
    -d ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${_generator} ${_generator_dependencies}
    COMMENT "Generating class ${_class_name} for ${_msg}"
    )
  set(${_files_generated} ${_h_file} ${_cxx_file} PARENT_SCOPE)
endfunction()


#
# convert std_msgs/msg/Float to StdMsgsFloat
#
function(ros2_msg_to_vtk_class _msg _result)
  # convert to class name, camel case
  # remove redundant /msg/
  string(REPLACE "/msg/" "_" _1 ${_msg})
  # replace _ by ; to make a list
  string(REPLACE "_" ";" _2 ${_1})
  set(_vtk "vtk")
  foreach(_3 ${_2})
    # extract first letter, toupper it and replace in original
    string(SUBSTRING ${_3} 0 1 _first)
    string(TOUPPER ${_first} _first)
    string(REGEX REPLACE "^.(.*)" "${_first}\\1" _4 "${_3}")
    string(APPEND _vtk ${_4})
  endforeach()
  # return result
  set(${_result} ${_vtk} PARENT_SCOPE)
endfunction()


#
# convert std_msgs/msg/Float to std_msgs::msg::Float
#
function(ros2_msg_to_ros_class _msg _result)
  string(REPLACE "/" "::" _1 ${_msg})
  set(${_result} ${_1} PARENT_SCOPE)
endfunction()


#
# convert std_msg/msg/Float to Float
#
function(ros2_msg_to_ros_short_class _msg _result)
  string(REPLACE "/" ";" _list ${_msg})
  list(GET _list -1 _class_name)
  set(${_result} ${_class_name} PARENT_SCOPE)
endfunction()


#
# create the code for a publisher, the result is two strings,
# header and implementation code
#
function(generate_ros2_publisher_code _msg _h _cxx _reg)
  ros2_msg_to_vtk_class(${_msg} vtk_class)
  ros2_msg_to_ros_class(${_msg} ros_class)
  ros2_msg_to_ros_short_class(${_msg} message_name)

  set(file_name "${vtk_class}.h")

  set(_h_tmp)
  string(APPEND _h_tmp "#include <${file_name}>\n")
  string(APPEND _h_tmp "VTK_MRML_ROS_PUBLISHER_VTK_H(${vtk_class}, ${message_name});\n")
  set(${_h} "${_h_tmp}" PARENT_SCOPE)

  set(_cxx_tmp)
  string(APPEND _cxx_tmp "VTK_MRML_ROS_PUBLISHER_VTK_CXX(${vtk_class}, ${ros_class}, ${message_name});\n")
  set(${_cxx} "${_cxx_tmp}" PARENT_SCOPE)

  set(_reg_tmp)
  string(APPEND _reg_tmp "  scene->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2Publisher${message_name}Node>::New());")
  set(${_reg} "${_reg_tmp}" PARENT_SCOPE)

endfunction()


#
# create the code for a subscriber, the result is two strings,
# header and implementation code
#
function(generate_ros2_subscriber_code _msg _h _cxx _reg)
  ros2_msg_to_vtk_class(${_msg} vtk_class)
  ros2_msg_to_ros_class(${_msg} ros_class)
  ros2_msg_to_ros_short_class(${_msg} message_name)

  set(file_name "${vtk_class}.h")

  set(_h_tmp)
  string(APPEND _h_tmp "#include <${file_name}>\n")
  string(APPEND _h_tmp "VTK_MRML_ROS_SUBSCRIBER_VTK_H(${vtk_class}, ${message_name});\n")
  set(${_h} "${_h_tmp}" PARENT_SCOPE)

  set(_cxx_tmp)
  string(APPEND _cxx_tmp "VTK_MRML_ROS_SUBSCRIBER_VTK_CXX(${ros_class}, ${vtk_class}, ${message_name});\n")
  set(${_cxx} "${_cxx_tmp}" PARENT_SCOPE)

  set(_reg_tmp)
  string(APPEND _reg_tmp "  scene->RegisterNodeClass(vtkSmartPointer<vtkMRMLROS2Subscriber${message_name}Node>::New());")
  set(${_reg} "${_reg_tmp}" PARENT_SCOPE)

endfunction()