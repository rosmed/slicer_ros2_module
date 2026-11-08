ctkMacroCompilePythonScript(
  TARGET_NAME SlicerROS2Python
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/slicerros2
  SCRIPTS
    __init__.py
    robots.py
    cameras.py
    dvrk.py
    examples/_moveit.py
    examples/point_cloud_publisher.py
    examples/point_cloud_subscriber.py
    examples/moveit_obstacle.py
    examples/moveit_custom_tool.py
    examples/moveit_pose_markers.py
    examples/__init__.py
    examples/patient_cart.py
    examples/ecm_camera.py
  DESTINATION_DIR ${CMAKE_BINARY_DIR}/${Slicer_QTSCRIPTEDMODULES_LIB_DIR}/slicerros2
  INSTALL_DIR ${Slicer_INSTALL_QTSCRIPTEDMODULES_LIB_DIR}
)
