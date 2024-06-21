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

  set (${_slicer_ros_GENERATED_FILES} ${_all_files_generated} PARENT_SCOPE)
endfunction()


#
# generate the vtk class corresponding to the ROS message as well as
# conversion methods between ROS and vtk
#
function(generate_ros2_message _msg _files_generated)
  # convert to class name, camel case
  # remove redundant /msg/
  string(REPLACE "/msg/" "_" _1 ${_msg})
  # replace _ by ; to make a list
  string(REPLACE "_" ";" _2 ${_1})
  set(_class_name "vtkROS2")
  foreach(_3 ${_2})
    # extract first letter, toupper it and replace in original
    string(SUBSTRING ${_3} 0 1 _first)
    string(TOUPPER ${_first} _first)
    string(REGEX REPLACE "^.(.*)" "${_first}\\1" _4 "${_3}")
    string(APPEND _class_name ${_4})
  endforeach()
  # create custom command
  set(_h "${CMAKE_CURRENT_BINARY_DIR}/${_class_name}.h")
  set(_cxx "${CMAKE_CURRENT_BINARY_DIR}/${_class_name}.cxx")
  set_source_files_properties(${_h} PROPERTIES GENERATED 1)
  set_source_files_properties(${_cxx} PROPERTIES GENERATED 1)
  set(_generator "${CMAKE_CURRENT_SOURCE_DIR}/CodeGeneration/ROS2_to_vtkObjects.py")
  add_custom_command(
    OUTPUT ${_h} ${_cxx}
    COMMAND ${_generator}
    -m ${_msg}
    -c ${_class_name}
    -d ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS "${_generator}"
    COMMENT "generating class ${_class_name} for ${_msg}"
    )
  set(${_files_generated} ${_cxx} ${_h} PARENT_SCOPE)
endfunction()




function(generate_publisher_node msg_type)
  string(REPLACE "/msg/" "_" msg_header ${msg_type})
  string(REPLACE "_" ";" msg_parts ${msg_header})
  list(GET msg_parts -1 message_name)

  string(REPLACE "/" "::" ros_message_type ${msg_type})

  set(class_name_stub "")
  foreach(part ${msg_parts})
    # extract first letter, toupper it and replace in original
    string(SUBSTRING ${part} 0 1 first_letter)
    string(TOUPPER ${first_letter} first_letter)
    string(REGEX REPLACE "^.(.*)" "${first_letter}\\1" capitalized_part "${part}")
    string(APPEND class_name_stub ${capitalized_part})
  endforeach()

  set(file_name "vtkROS2${class_name_stub}.h")
  set(class_name "vtk${class_name_stub}")

  message("Generating node of type PUBLISHER with message ${msg_type}")
  message("Filename: ${file_name}") # vtkROS2GeometryMsgsPoseStamped.h
  message("Class name: ${class_name}") # vtkGeometryMsgsPoseStamped
  message("Message name: ${message_name}") # PoseStamped
  message("ROS message type: ${ros_message_type}") # geometry_msgs::msg::PoseStamped

  # Read the contents of the file into a variable
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/vtkMRMLROS2PublisherDefaultNodes.h" FILE_CONTENTS)

    string(APPEND FILE_CONTENTS "\n// Additional lines appended by CMake\n")
    string(APPEND FILE_CONTENTS "//#include <${file_name}>\n")
    string(APPEND FILE_CONTENTS "//VTK_MRML_ROS_PUBLISHER_VTK_H(${class_name},${message_name})\n")

  file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/vtkMRMLROS2PublisherDefaultNodes.h" "${FILE_CONTENTS}")

  # Read the contents of the file into a variable
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/vtkMRMLROS2PublisherDefaultNodes.cxx" FILE_CONTENTS)

    string(APPEND FILE_CONTENTS "\n// Additional lines appended by CMake\n")
    string(APPEND FILE_CONTENTS "//VTK_MRML_ROS_PUBLISHER_VTK_CXX(${class_name},${ros_message_type},${message_name});\n")

  file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/vtkMRMLROS2PublisherDefaultNodes.cxx" "${FILE_CONTENTS}")

endfunction()

function(generate_subscriber_node msg_type)
  message("Generating node of type SUBSCRIBER with message ${msg_type}")
endfunction()

function(generate_service_node msg_type)
  message("Generating node of type SERVICE with message ${msg_type}")
endfunction()
