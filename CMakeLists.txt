# SlicerConfig.cmake does god knows what to the build tree but it
# needs to be the first thing we do
find_package(Slicer REQUIRED COMPONENTS ConfigurePrerequisites)
cmake_minimum_required(VERSION 3.5)
project(SlicerRos2)

#---------------------------------------------------------------------------
find_package(orocos_kdl REQUIRED)
IF (orocos_kdl_FOUND)
	message(STATUS "orocos_kdl found. Version: ${orocos_kdl_VERSION}")
  message(STATUS "kdl lib dir: ${orocos_kdl_LIBRARY_DIR}")
	include_directories(BEFORE SYSTEM ${orocos_kdl_INCLUDE_DIRS})
	add_definitions(${orocos_kdl_CFLAGS})
	link_directories(${orocos_kdl_LIBRARY_DIR})
ENDIF()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
#-----------------------------------------------------------------------------

set(MODULE_NAME Ros2)
set(MODULE_TITLE ${MODULE_NAME})

string(TOUPPER ${MODULE_NAME} MODULE_NAME_UPPER)

# Now, we really find Slicer package but we need to make sure we unset
# PYTHON_EXECUTABLE (likely set by ROS 2) so Slicer doesn't complain
# (note that most Ubuntu and ROS 2 python packages will not be usable in
# Slicer)
unset(PYTHON_EXECUTABLE CACHE)
find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})

# Set after including Slicer to avoid errors during CMake step.  This
# is for ROS Galactic, Foxy uses CXX 14...
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#---------------------------------------------------------------------------

add_subdirectory(Logic)
link_libraries(stdc++fs)

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
  )

set(MODULE_TARGET_LIBRARIES
  vtkSlicer${MODULE_NAME}ModuleLogic
  )

set(MODULE_RESOURCES
  Resources/qSlicer${MODULE_NAME}Module.qrc
  )

#-----------------------------------------------------------------------------
slicerMacroBuildLoadableModule(
  NAME ${MODULE_NAME}
  TITLE ${MODULE_TITLE}
  EXPORT_DIRECTIVE ${MODULE_EXPORT_DIRECTIVE}
  INCLUDE_DIRECTORIES ${MODULE_INCLUDE_DIRECTORIES}
  SRCS ${MODULE_SRCS}
  MOC_SRCS ${MODULE_MOC_SRCS}
  UI_SRCS ${MODULE_UI_SRCS}
  TARGET_LIBRARIES ${MODULE_TARGET_LIBRARIES}  ${kdl_parser_LIBRARIES} ${urdf_LIBRARIES}
  RESOURCES ${MODULE_RESOURCES}
  WITH_GENERIC_TESTS
  )

#-----------------------------------------------------------------------------
if(BUILD_TESTING)
  add_subdirectory(Testing)
endif()
