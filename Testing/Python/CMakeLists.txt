cmake_minimum_required(VERSION 3.5)
project(SlicerROS2Tests)

# quick and dirty hack to copy the test file in the same directory as
# the C++ runtime libraries

get_target_property(output_dir qSlicer${MODULE_NAME}Module LIBRARY_OUTPUT_DIRECTORY)

set(PY_TEST_FILE_DEST ${output_dir}/SlicerROS2Tests.py)
set(PY_TEST_FILE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/SlicerROS2Tests.py)
add_custom_target(copy-test-module ALL DEPENDS ${PY_TEST_FILE_DEST})
add_custom_command(
  OUTPUT ${PY_TEST_FILE_DEST}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PY_TEST_FILE_SRC} ${PY_TEST_FILE_DEST}
  DEPENDS ${PY_TEST_FILE_SRC})
