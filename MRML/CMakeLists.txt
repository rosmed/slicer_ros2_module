project(vtkSlicer${MODULE_NAME}ModuleMRML)

set(KIT ${PROJECT_NAME})

set(${KIT}_EXPORT_DIRECTIVE "VTK_SLICER_${MODULE_NAME_UPPER}_MODULE_MRML_EXPORT")

set(${KIT}_INCLUDE_DIRECTORIES
  ${Slicer_Base_INCLUDE_DIRS}
  )

# --------------------------------------------------------------------------
# Sources

set(_SRCS_INTERNAL
  # conversion methods
  vtkROS2ToSlicer.h
  vtkROS2ToSlicer.cxx
  vtkSlicerToROS2.h
  vtkSlicerToROS2.cxx
  # node internals
  vtkMRMLROS2NodeInternals.h
  vtkMRMLROS2PublisherInternals.h
  vtkMRMLROS2SubscriberInternals.h
  )

set_source_files_properties(
  ${_SRCS_INTERNAL}
  PROPERTIES
  WRAP_EXCLUDE_PYTHON 1
  )

set(${KIT}_SRCS
  # private classes
  ${_SRCS_INTERNAL}
  # public classes
  vtkMRMLROS2NODENode.h
  vtkMRMLROS2NODENode.cxx
  vtkMRMLROS2SubscriberNode.h
  vtkMRMLROS2SubscriberNode.cxx
  vtkMRMLROS2SubscriberDefaultNodes.h
  vtkMRMLROS2SubscriberDefaultNodes.cxx
  vtkMRMLROS2PublisherNode.h
  vtkMRMLROS2PublisherNode.cxx
  vtkMRMLROS2PublisherDefaultNodes.h
  vtkMRMLROS2PublisherDefaultNodes.cxx
  )

SET (${KIT}_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "" FORCE)

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
  TARGET_LIBRARIES ${${KIT}_TARGET_LIBRARIES} ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES}
  )
