macro (undefine_python)
  unset(PYTHON_EXECUTABLE CACHE)
  unset(Python3_EXECUTABLE CACHE)
  unset(PYTHON_EXECUTABLE)
  unset(Python3_EXECUTABLE)
endmacro ()

# Before Slicer does anything, let's find the default Python
find_package (Python3 COMPONENTS Interpreter)
if (Python3_Interpreter_FOUND)
  if (DEFINED CACHE{ROS_Python3_EXECUTABLE}})
  else ()
    set (ROS_Python3_EXECUTABLE ${Python3_EXECUTABLE} CACHE INTERNAL "Python interpreter for ROS 2")
  endif ()
endif ()

undefine_python ()

# SlicerConfig.cmake does god knows what to the build tree but it
# needs to be the first thing we do
find_package(Slicer REQUIRED COMPONENTS ConfigurePrerequisites)
cmake_minimum_required(VERSION 3.5)
project(slicer_ros2_module)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rqt_gui_cpp REQUIRED)

include_directories (${urdf_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS})

#-----------------------------------------------------------------------------

set(MODULE_NAME ROS2)
set(MODULE_TITLE ${MODULE_NAME})

string(TOUPPER ${MODULE_NAME} MODULE_NAME_UPPER)

# Now, we really find Slicer package but we need to make sure we unset
# PYTHON_EXECUTABLE (likely set by ROS 2) so Slicer doesn't complain
# (note that most Ubuntu and ROS 2 python packages will not be usable in
# Slicer)
undefine_python ()

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
  TARGET_LIBRARIES ${MODULE_TARGET_LIBRARIES}  ${kdl_parser_LIBRARIES} ${urdf_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_LIBRARIES} ${rqt_gui_cpp_LIBRARIES}
  RESOURCES ${MODULE_RESOURCES}
  WITH_GENERIC_TESTS
  )

#-----------------------------------------------------------------------------
# hack to copy python test script, not the proper place to copy this!
set (PY_TEST_FILE_DEST ${Slicer_DIR}/bin/Python/slicer_ros2_module_test.py)
set (PY_TEST_FILE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/slicer_ros2_module_test.py)
add_custom_target(copy-test-file ALL DEPENDS ${PY_TEST_FILE_DEST})
add_custom_command(
  OUTPUT ${PY_TEST_FILE_DEST}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PY_TEST_FILE_SRC} ${PY_TEST_FILE_DEST}
  DEPENDS ${PY_TEST_FILE_SRC})

#-----------------------------------------------------------------------------
# hack to copy custom slicer startup script
set (PY_STARTUP_FILE_DEST ${Slicer_DIR}/.slicerrc.py)
set (PY_STARTUP_FILE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/slicer_startup_script.py)
add_custom_target(copy-startup-file ALL DEPENDS ${PY_STARTUP_FILE_DEST})
add_custom_command(
  OUTPUT ${PY_STARTUP_FILE_DEST}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PY_STARTUP_FILE_SRC} ${PY_STARTUP_FILE_DEST}
  DEPENDS ${PY_STARTUP_FILE_SRC})

#-----------------------------------------------------------------------------
if(BUILD_TESTING)
  add_subdirectory(Testing)
endif()

# Not sure how stable this code is.  Basically the CMake code for
# Slicer sets a default Python interpreter so we need to revert to the
# "default" one
set(Python3_EXECUTABLE ${ROS_Python3_EXECUTABLE} CACHE STRING "ROS Python" FORCE)
set(PYTHON_EXECUTABLE ${ROS_Python3_EXECUTABLE} CACHE STRING "ROS Python" FORCE)

ament_package()
