cmake_minimum_required(VERSION 3.5)
project(ROS2Tests)

# quick and dirty hack to copy the test file in the same directory as
# the C++ runtime libraries

set (PY_TEST_FILE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/ROS2Tests.py)

install (
  FILES ${PY_TEST_FILE_SRC}
  DESTINATION  "${Slicer_DIR}/${Slicer_QTLOADABLEMODULES_BIN_DIR}")
