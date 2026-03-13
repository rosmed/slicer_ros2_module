project(vtkSlicer${MODULE_NAME}ModuleMRML)

set(KIT ${PROJECT_NAME})

set(${KIT}_EXPORT_DIRECTIVE "VTK_SLICER_${MODULE_NAME_UPPER}_MODULE_MRML_EXPORT")

set(${KIT}_INCLUDE_DIRECTORIES
  ${Slicer_Base_INCLUDE_DIRS}
  ${cisst_msgs_INCLUDE_DIRS}
  ${turtlesim_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  )

# include SlicerROS2 for code generation functions
include(SlicerROS2.cmake)

generate_ros2_nodes(
  GENERATED_FILES_PREFIX
  "SLICER_ROS2_GENERATED"

  PUBLISHERS
  "geometry_msgs/msg/PoseStamped"
  "geometry_msgs/msg/TransformStamped"
  "geometry_msgs/msg/WrenchStamped"
  "sensor_msgs/msg/Joy"
  "sensor_msgs/msg/JointState"
  "geometry_msgs/msg/PoseArray"

  SUBSCRIBERS
  "geometry_msgs/msg/PoseStamped"
  "geometry_msgs/msg/TransformStamped"
  "geometry_msgs/msg/WrenchStamped"
  "sensor_msgs/msg/Joy"
  "sensor_msgs/msg/JointState"
  "geometry_msgs/msg/PoseArray"

  SERVICE_CLIENTS
  "rosbag2_interfaces/srv/Burst"
  "rosbag2_interfaces/srv/GetRate"
  "rosbag2_interfaces/srv/IsPaused"
  "rosbag2_interfaces/srv/Pause"
  # "rosbag2_interfaces/srv/Play"
  "rosbag2_interfaces/srv/PlayNext"
  "rosbag2_interfaces/srv/Resume"
  "rosbag2_interfaces/srv/Seek"
  "rosbag2_interfaces/srv/SetRate"
  "rosbag2_interfaces/srv/Snapshot"
  # "rosbag2_interfaces/srv/SplitBagfile"
  # "rosbag2_interfaces/srv/Stop"
  "rosbag2_interfaces/srv/TogglePaused"

  "moveit_msgs/srv/GetPositionFK"
  "moveit_msgs/srv/GetPositionIK"
  "moveit_msgs/srv/GetCartesianPath"
  "turtlesim/srv/Spawn"

  DEPENDENCIES
  # messages used by other messages for which we need to
  # create a VTK object
  "std_msgs/msg/Header"
  "builtin_interfaces/msg/Time"
  "builtin_interfaces/msg/Duration"
  "sensor_msgs/msg/MultiDOFJointState"
  "geometry_msgs/msg/Polygon"
  "geometry_msgs/msg/Point32"
  "geometry_msgs/msg/Point"
  "geometry_msgs/msg/Vector3"
  "geometry_msgs/msg/Quaternion"
  "shape_msgs/msg/SolidPrimitive"
  "shape_msgs/msg/Mesh"
  "shape_msgs/msg/MeshTriangle"
  "shape_msgs/msg/Plane"
  "object_recognition_msgs/msg/ObjectType"
  "trajectory_msgs/msg/JointTrajectoryPoint"
  "trajectory_msgs/msg/JointTrajectory"
  "trajectory_msgs/msg/MultiDOFJointTrajectory"
  "trajectory_msgs/msg/MultiDOFJointTrajectoryPoint"
  "moveit_msgs/msg/RobotTrajectory"
  "moveit_msgs/msg/RobotState"
  "moveit_msgs/msg/MoveItErrorCodes"
  "moveit_msgs/msg/CollisionObject"
  "moveit_msgs/msg/AttachedCollisionObject"
  "moveit_msgs/msg/Constraints"
  "moveit_msgs/msg/JointConstraint"
  "moveit_msgs/msg/PositionConstraint"
  "moveit_msgs/msg/PositionIKRequest"
  "moveit_msgs/msg/BoundingVolume"
  "moveit_msgs/msg/OrientationConstraint"
  "moveit_msgs/msg/VisibilityConstraint"
  )

get_directory_property(_current_prop ADDITIONAL_CLEAN_FILES)
set_directory_properties(PROPERTIES ADDITIONAL_CLEAN_FILES "${SLICER_ROS2_GENERATED_H};${SLICER_ROS2_GENERATED_CXX};${_current_prop}")

set(_SRCS_INTERNAL
  # conversion methods
  # vtkROS2ToSlicer.h
  vtkROS2ToSlicer.cxx
  # vtkSlicerToROS2.h
  vtkSlicerToROS2.cxx
  # vtkMRMLROS2Utils.h
  vtkMRMLROS2Utils.cxx
  )

set(${KIT}_SRCS
  # private classes
  ${_SRCS_INTERNAL}
  # generated classes
  ${SLICER_ROS2_GENERATED_H}
  ${SLICER_ROS2_GENERATED_CXX}
  # public classes
  vtkMRMLROS2NodeNode.h
  vtkMRMLROS2NodeNode.cxx
  vtkMRMLROS2SubscriberNode.h
  vtkMRMLROS2SubscriberNode.cxx
  vtkMRMLROS2SubscriberDefaultNodes.h
  vtkMRMLROS2SubscriberDefaultNodes.cxx
  vtkMRMLROS2PublisherNode.h
  vtkMRMLROS2PublisherNode.cxx
  vtkMRMLROS2PublisherDefaultNodes.h
  vtkMRMLROS2PublisherDefaultNodes.cxx
  vtkMRMLROS2ParameterNode.h
  vtkMRMLROS2ParameterNode.cxx
  vtkMRMLROS2Tf2BroadcasterNode.h
  vtkMRMLROS2Tf2BroadcasterNode.cxx
  vtkMRMLROS2Tf2LookupNode.h
  vtkMRMLROS2Tf2LookupNode.cxx
  vtkMRMLROS2RobotNode.h
  vtkMRMLROS2RobotNode.cxx
  vtkMRMLROS2ServiceClientNode.h
  vtkMRMLROS2ServiceClientNode.cxx
  vtkMRMLROS2ServiceClientDefaultNodes.h
  vtkMRMLROS2ServiceClientDefaultNodes.cxx
  vtkCustomTypes.h
  vtkCustomTypes.cxx
  )

set (${KIT}_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "" FORCE)

# --------------------------------------------------------------------------
# Build the library

set(${KIT}_TARGET_LIBRARIES
  ${MRML_LIBRARIES}
  SlicerBaseLogic
  ${SlicerROS2_ROS_LIBRARIES}
  /opt/ros/humble/lib/libmoveit_move_group_interface.so.2.5.9
  /opt/ros/humble/lib/libmoveit_robot_model_loader.so.2.5.9
  ${kdl_parser_LIBRARIES}
  kdl_parser::kdl_parser
  )

SlicerMacroBuildModuleMRML(
  NAME ${KIT}
  EXPORT_DIRECTIVE ${${KIT}_EXPORT_DIRECTIVE}
  INCLUDE_DIRECTORIES ${${KIT}_INCLUDE_DIRECTORIES}
  SRCS ${${KIT}_SRCS}
  TARGET_LIBRARIES ${${KIT}_TARGET_LIBRARIES}
  )
