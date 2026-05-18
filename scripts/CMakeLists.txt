# Configure the extension manager script with this build/install context.
configure_file(
  manage_extensions.py.in
  manage_extensions.py
)

install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/manage_extensions.py
  demo_point_cloud.py
  slicer
  test
  DESTINATION lib/${PROJECT_NAME}
  COMPONENT slicer_ros2_scripts
)

add_custom_target(install_slicer_scripts
  COMMAND ${CMAKE_COMMAND} --install "${CMAKE_BINARY_DIR}" --component slicer_ros2_scripts
  COMMENT "Installing slicer_ros2_module scripts"
)
