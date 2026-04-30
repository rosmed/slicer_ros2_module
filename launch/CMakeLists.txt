# Configure the Slicer launcher with the Slicer build directory path.
configure_file(
  slicer.launch.py.in
  slicer.launch.py
)

# Install static launch files (ur_sim_control.launch.py).
# Exclude templates to avoid polluting the install space with .in files.
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION share/${PROJECT_NAME}/launch
  COMPONENT slicer_ros2_launch
  PATTERN "*.in" EXCLUDE
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install generated launch file (slicer.launch.py).
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/slicer.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
  COMPONENT slicer_ros2_launch
)

add_custom_target(install_slicer_launch
  COMMAND ${CMAKE_COMMAND} --install "${CMAKE_BINARY_DIR}" --component slicer_ros2_launch
  COMMENT "Installing slicer_ros2_module launch files"
)
