project(vtkSlicer${MODULE_NAME}ModuleLogic)

set(KIT ${PROJECT_NAME})

set(${KIT}_EXPORT_DIRECTIVE "VTK_SLICER_${MODULE_NAME_UPPER}_MODULE_LOGIC_EXPORT")

set(${KIT}_INCLUDE_DIRECTORIES
  )

set(${KIT}_SRCS
  vtkSlicer${MODULE_NAME}Logic.cxx
  vtkSlicer${MODULE_NAME}Logic.h
  # conversion methods
  vtkROS2ToSlicer.h
  vtkROS2ToSlicer.cxx
  # public classes
  vtkMRMLROS2NodeNode.h
  vtkMRMLROS2NodeNode.cxx
  vtkMRMLROS2SubscriberNode.h
  vtkMRMLROS2SubscriberNode.cxx
  vtkMRMLROS2SubscriberNativeNode.h
  vtkMRMLROS2SubscriberNativeNode.cxx
  vtkMRMLROS2SubscriberVTKNode.h
  vtkMRMLROS2SubscriberVTKNode.cxx
  )

set(${KIT}_TARGET_LIBRARIES
  ${ITK_LIBRARIES}
  )

#-----------------------------------------------------------------------------
SlicerMacroBuildModuleLogic(
  NAME ${KIT}
  EXPORT_DIRECTIVE ${${KIT}_EXPORT_DIRECTIVE}
  INCLUDE_DIRECTORIES ${${KIT}_INCLUDE_DIRECTORIES}
  SRCS ${${KIT}_SRCS}
  TARGET_LIBRARIES ${${KIT}_TARGET_LIBRARIES} ${kdl_parser_LIBRARIES} ${urdf_LIBRARIES} ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES}  ${tf2_ros_LIBRARIES} ${tf2_msgs_LIBRARIES} ${tf2_LIBRARIES}
  )
