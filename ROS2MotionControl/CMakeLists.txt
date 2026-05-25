#-----------------------------------------------------------------------------
set(MODULE_NAME ROS2MotionControl)

#-----------------------------------------------------------------------------
set(MODULE_PYTHON_SCRIPTS
  ${MODULE_NAME}.py
  )

# TrajectoryGenerators.py is a plain Python library, not a scripted module.
# Install it to a subdirectory so Slicer does not try to load it as a module.
install(FILES TrajectoryGenerators.py
  DESTINATION ${Slicer_INSTALL_QTSCRIPTEDMODULES_LIB_DIR}/ROS2MotionControl
  COMPONENT RuntimeLibraries
  )

set(MODULE_PYTHON_RESOURCES
  Resources/Icons/${MODULE_NAME}.png
  Resources/UI/${MODULE_NAME}.ui
  )

#-----------------------------------------------------------------------------
slicerMacroBuildScriptedModule(
  NAME ${MODULE_NAME}
  SCRIPTS ${MODULE_PYTHON_SCRIPTS}
  RESOURCES ${MODULE_PYTHON_RESOURCES}
  )
