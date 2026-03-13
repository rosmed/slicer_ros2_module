macro (undefine_python)
  unset(PYTHON_EXECUTABLE CACHE)
  unset(Python3_EXECUTABLE CACHE)
  unset(PYTHON_EXECUTABLE)
  unset(Python3_EXECUTABLE)
endmacro ()

# Set CMake policy to handle empty list elements (fixes Python FindPython error)
if(POLICY CMP0007)
  cmake_policy(SET CMP0007 NEW)
endif()

# Before Slicer does anything, let's find the default Python
find_package (Python3 COMPONENTS Interpreter)
if (Python3_Interpreter_FOUND)
  if (DEFINED ROS_Python3_EXECUTABLE)
    # Use existing cached ROS_Python3_EXECUTABLE
  else ()
    set (ROS_Python3_EXECUTABLE ${Python3_EXECUTABLE} CACHE INTERNAL "Python interpreter for ROS 2")
  endif ()
endif ()

# unset settings so we can go back and forth between Slicer vs ROS2 preferences
undefine_python ()
unset (CMAKE_INSTALL_PREFIX)

# SlicerConfig.cmake does god knows what to the build tree but it
# needs to be the first thing we do
find_package(Slicer REQUIRED COMPONENTS ConfigurePrerequisites)
cmake_minimum_required(VERSION 3.5)
project(slicer_ros2_module)

option (USE_CISST_MSGS "Add topics for cisst messages (https://github.com/jhu-cisst/ros2_cisst_msgs)" OFF)

configure_file (
  "${slicer_ros2_module_SOURCE_DIR}/SlicerROS2Config.h.in"
  "${slicer_ros2_module_BINARY_DIR}/include/SlicerROS2Config.h")
include_directories ("${slicer_ros2_module_BINARY_DIR}/include")

# set list of required packages
set (
  SlicerROS2_ROS_DEPENDENCIES
  # core
  ament_cmake
  rclcpp
  kdl_parser
  urdf
  tf2
  tf2_ros
  turtlesim
  # messages and services
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  object_recognition_msgs
  trajectory_msgs
  moveit_msgs
  rosbag2_interfaces
  moveit_ros_planning
  moveit_core
  moveit_ros_planning_interface
  # KDL solver
  orocos_kdl
)

set (SlicerROS2_ROS_LIBRARIES "")

foreach (dep ${SlicerROS2_ROS_DEPENDENCIES})
  find_package(${dep} REQUIRED)
  include_directories (${${dep}_INCLUDE_DIRS})
  set (SlicerROS2_ROS_LIBRARIES ${SlicerROS2_ROS_LIBRARIES} ${${dep}_LIBRARIES})
endforeach ()

# Explicitly add kdl_parser libraries
if (kdl_parser_FOUND)
  set (SlicerROS2_ROS_LIBRARIES ${SlicerROS2_ROS_LIBRARIES} ${kdl_parser_LIBRARIES})
endif ()

# preserve ament install prefix to create a dummy setup.bash
if (NOT DEFINED SlicerROS2_AMENT_INSTALL_PREFIX)
  set (SlicerROS2_AMENT_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE FILEPATH "ament install prefix")
endif ()

install(
  FILES local_setup.bash
  DESTINATION /${SlicerROS2_AMENT_INSTALL_PREFIX}/share/slicer_ros2_module)

#-----------------------------------------------------------------------------

set(MODULE_NAME ROS2)
set(MODULE_TITLE ${MODULE_NAME})

string(TOUPPER ${MODULE_NAME} MODULE_NAME_UPPER)

# Now, we really find Slicer package but we need to make sure we unset
# PYTHON_EXECUTABLE (likely set by ROS 2) so Slicer doesn't complain
# (note that most Ubuntu and ROS 2 python packages will not be usable in
# Slicer)
undefine_python ()

# Unset CURL variables set by ROS2 packages that conflict with Slicer's bundled CURL
unset(CURL_INCLUDE_DIR CACHE)
unset(CURL_LIBRARY CACHE)
unset(CURL_INCLUDE_DIR)
unset(CURL_LIBRARY)

find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})

# Set after including Slicer to avoid errors during CMake step.  This
# is for ROS Galactic, Foxy uses CXX 14...
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#---------------------------------------------------------------------------

add_subdirectory(MRML)
add_subdirectory(Logic)

#-----------------------------------------------------------------------------
set(MODULE_EXPORT_DIRECTIVE "Q_SLICER_QTMODULES_${MODULE_NAME_UPPER}_EXPORT")

# Current_{source,binary} and Slicer_{Libs,Base} already included
set(MODULE_INCLUDE_DIRECTORIES
  ${CMAKE_CURRENT_SOURCE_DIR}/Logic
  ${CMAKE_CURRENT_BINARY_DIR}/Logic
  ${CMAKE_CURRENT_SOURCE_DIR}/Widgets
  ${CMAKE_CURRENT_BINARY_DIR}/Widgets
  )

set(MODULE_SRCS
  qSlicer${MODULE_NAME}Module.cxx
  qSlicer${MODULE_NAME}Module.h
  qSlicer${MODULE_NAME}ModuleWidget.cxx
  qSlicer${MODULE_NAME}ModuleWidget.h
  )

set(MODULE_MOC_SRCS
  qSlicer${MODULE_NAME}Module.h
  qSlicer${MODULE_NAME}ModuleWidget.h
  )

set(MODULE_UI_SRCS
  Resources/UI/qSlicer${MODULE_NAME}ModuleWidget.ui
  Resources/UI/qSlicer${MODULE_NAME}RobotWidget.ui
  )

set(MODULE_TARGET_LIBRARIES
  vtkSlicer${MODULE_NAME}ModuleLogic
  )

set(MODULE_RESOURCES
  Resources/qSlicer${MODULE_NAME}Module.qrc
  )

set(CMAKE_INSTALL_PREFIX ${Slicer_DIR} CACHE PATH "Path used to install loadable modules" FORCE)

#-----------------------------------------------------------------------------
slicerMacroBuildLoadableModule(
  NAME ${MODULE_NAME}
  TITLE ${MODULE_TITLE}
  EXPORT_DIRECTIVE ${MODULE_EXPORT_DIRECTIVE}
  INCLUDE_DIRECTORIES ${MODULE_INCLUDE_DIRECTORIES}
  SRCS ${MODULE_SRCS}
  MOC_SRCS ${MODULE_MOC_SRCS}
  UI_SRCS ${MODULE_UI_SRCS}
  TARGET_LIBRARIES ${MODULE_TARGET_LIBRARIES}  ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES} ${urdf_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_LIBRARIES} ${rqt_gui_cpp_LIBRARIES}
  RESOURCES ${MODULE_RESOURCES}
  WITH_GENERIC_TESTS
  )

if(BUILD_TESTING)
  add_subdirectory(Testing)
endif()

# Not sure how stable this code is.  Basically the CMake code for
# Slicer sets a default Python interpreter so we need to revert to the
# "default" one
set(Python3_EXECUTABLE ${ROS_Python3_EXECUTABLE} CACHE STRING "ROS Python" FORCE)
set(PYTHON_EXECUTABLE ${ROS_Python3_EXECUTABLE} CACHE STRING "ROS Python" FORCE)

# Slicer macro will change the project name to ROS2 and ament_package
# will report this as an error
project(slicer_ros2_module)
ament_package()
