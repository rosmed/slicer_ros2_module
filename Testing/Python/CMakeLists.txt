cmake_minimum_required(VERSION 3.5)

set(MODULE_NAME ROS2Tests)

slicerMacroBuildScriptedModule(
  NAME ${MODULE_NAME}
  SCRIPTS ${MODULE_NAME}.py
)
