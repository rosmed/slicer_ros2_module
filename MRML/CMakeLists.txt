project(vtkSlicer${MODULE_NAME}ModuleMRML)

set(KIT ${PROJECT_NAME})

set(${KIT}_EXPORT_DIRECTIVE "VTK_SLICER_${MODULE_NAME_UPPER}_MODULE_MRML_EXPORT")

set(${KIT}_INCLUDE_DIRECTORIES
  ${Slicer_Base_INCLUDE_DIRS}
  ${cisst_msgs_INCLUDE_DIRS} 
  ${turtlesim_INCLUDE_DIRS} 
  ${std_srvs_INCLUDE_DIRS} 
  )

# --------------------------------------------------------------------------
# Sources

# SLICER_ROS (
#   PUBLISHERS "geometry_msgs/msg/PoseStamped"
#   SUBSCRIBERS "geometry_msgs/msg/WrenchStamped"
#   SERVICES "std_srvs/srv/Empty"
#   )
# macro will:
# 1. List all ROS mssages usedand create conversion methods as needed
# 2. Create a ROS2 MRML node for each publisher, subscriber, service
# 2.1. Create a header + cpp file for each node using existing C++  macros
# 2.2. Add the node to the RegisterNode of logic using CMAKE configure_file



### Include the new cmake file with the macros instead of having the full logic here



set(_ROS_MESSAGES
  "geometry_msgs/msg/PoseStamped"
  "sensor_msgs/msg/JointState"
  "sensor_msgs/msg/Joy"
  "geometry_msgs/msg/WrenchStamped"
  "std_msgs/msg/Header"
  "builtin_interfaces/msg/Time"
  )

foreach(_msg ${_ROS_MESSAGES})
  # convert to class name, camel case
  # remove redundant /msg/
  string(REPLACE "/msg/" "_" _1 ${_msg})
  # replace _ by ; to make a list
  string(REPLACE "_" ";" _2 ${_1})
  set (_class_name "vtkROS2")
  foreach (_3 ${_2})
    # extract first letter, toupper it and replace in original
    string(SUBSTRING ${_3} 0 1 _first)
    string(TOUPPER ${_first} _first)
    string(REGEX REPLACE "^.(.*)" "${_first}\\1" _4 "${_3}")
    string(APPEND _class_name ${_4})
  endforeach ()

  # create custom command
  set(_h "${CMAKE_CURRENT_BINARY_DIR}/${_class_name}.h")
  set(_cxx "${CMAKE_CURRENT_BINARY_DIR}/${_class_name}.cxx")
  set_source_files_properties(${_h} PROPERTIES GENERATED 1)
  set_source_files_properties(${_cxx} PROPERTIES GENERATED 1)
  set(_generator "${CMAKE_CURRENT_SOURCE_DIR}/ROS2_to_vtkObjects.py")
  add_custom_command (
    OUTPUT ${_h} ${_cxx}
    COMMAND ${_generator}
    -m ${_msg}
    -c ${_class_name}
    -d ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS "${_generator}"
    COMMENT "generating class ${_class_name} for ${_msg}")

  set (_SRCS_GENERATED ${_SRCS_GENERATED} ${_cxx} ${_h})
endforeach()

set(_SRCS_INTERNAL
  # conversion methods
  # vtkROS2ToSlicer.h
  vtkROS2ToSlicer.cxx
  # vtkSlicerToROS2.h
  vtkSlicerToROS2.cxx
  # vtkMRMLROS2Utils.h
  vtkMRMLROS2Utils.cxx
  )

set(${KIT}_SRCS
  # private classes
  ${_SRCS_INTERNAL}
  # generated classes
  ${_SRCS_GENERATED}
  # public classes
  vtkMRMLROS2NodeNode.h
  vtkMRMLROS2NodeNode.cxx
  vtkMRMLROS2SubscriberNode.h
  vtkMRMLROS2SubscriberNode.cxx
  vtkMRMLROS2SubscriberDefaultNodes.h
  vtkMRMLROS2SubscriberDefaultNodes.cxx
  vtkMRMLROS2PublisherNode.h
  vtkMRMLROS2PublisherNode.cxx
  vtkMRMLROS2PublisherDefaultNodes.h
  vtkMRMLROS2PublisherDefaultNodes.cxx
  vtkMRMLROS2ParameterNode.h
  vtkMRMLROS2ParameterNode.cxx
  vtkMRMLROS2Tf2BroadcasterNode.h
  vtkMRMLROS2Tf2BroadcasterNode.cxx
  vtkMRMLROS2Tf2LookupNode.h
  vtkMRMLROS2Tf2LookupNode.cxx
  vtkMRMLROS2RobotNode.h
  vtkMRMLROS2RobotNode.cxx
  vtkMRMLROS2ServiceNode.h
  vtkMRMLROS2ServiceNode.cxx
  vtkMRMLROS2ServiceClientNode.h
  vtkMRMLROS2ServiceClientNode.cxx
  vtkMRMLROS2ServiceClientDefaultNodes.h
  vtkMRMLROS2ServiceClientDefaultNodes.cxx
  vtkCustomTypes.h
  vtkCustomTypes.cxx
  )

if (USE_CISST_MSGS)
  set (${KIT}_SRCS
    ${${KIT}_SRCS}
    vtkMRMLROS2CISST.h
    vtkMRMLROS2CISST.cxx)
endif ()

set (${KIT}_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "" FORCE)

# --------------------------------------------------------------------------
# Build the library

set(${KIT}_TARGET_LIBRARIES
  ${MRML_LIBRARIES}
  SlicerBaseLogic
  )

SlicerMacroBuildModuleMRML(
  NAME ${KIT}
  EXPORT_DIRECTIVE ${${KIT}_EXPORT_DIRECTIVE}
  INCLUDE_DIRECTORIES ${${KIT}_INCLUDE_DIRECTORIES}
  SRCS ${${KIT}_SRCS}
  TARGET_LIBRARIES ${${KIT}_TARGET_LIBRARIES} ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_msgs_LIBRARIES} ${tf2_LIBRARIES} ${urdf_LIBRARIES} ${cisst_msgs_LIBRARIES} ${turtlesim_LIBRARIES} ${std_srvs_LIBRARIES} 
  )
