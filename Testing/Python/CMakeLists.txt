cmake_minimum_required(VERSION 3.5)

set(MODULE_NAME ROS2Tests)

slicerMacroBuildScriptedModule(
  NAME ${MODULE_NAME}
  SCRIPTS ${MODULE_NAME}.py
  WITH_GENERIC_TESTS
)

# Register with CTest. --no-main-window runs Slicer headlessly (no Qt window)
# but the full MRML/VTK/Python stack and event loop are still active.
# Xvfb is still needed in CI for ROS2 subprocess nodes (e.g. turtlesim).
slicer_add_python_unittest(
  SCRIPT ${MODULE_NAME}.py
  SLICER_ARGS --no-main-window
)
