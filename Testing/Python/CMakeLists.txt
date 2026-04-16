cmake_minimum_required(VERSION 3.5)

set(MODULE_NAME ROS2Tests)

slicerMacroBuildScriptedModule(
  NAME ${MODULE_NAME}
  SCRIPTS ${MODULE_NAME}.py
  WITH_GENERIC_TESTS
)

# Also add the unittest so Slicer's CTest sees it
slicer_add_python_unittest(SCRIPT ${MODULE_NAME}.py)
