from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # UR control with fake hardware — no real robot or URSim needed
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': '192.168.56.101',
            'use_mock_hardware': 'true',
            'launch_rviz': launch_rviz,
        }.items()
    )

    # MoveIt
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'launch',
                'ur_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': launch_rviz,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5',
            description='UR robot type (ur3, ur5, ur10, ur16)'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Launch RViz alongside MoveIt'
        ),

        driver_launch,
        moveit_launch,
    ])
