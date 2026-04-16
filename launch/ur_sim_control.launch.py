from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5',
            description='UR robot type (ur3, ur5, ur10, ur16)'
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.56.101',
            description='IP address of the robot/simulator'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Launch RViz'
        ),

        # Step 1: Start URSim (using ros2 run as requested in docs)
        ExecuteProcess(
            cmd=['ros2', 'run', 'ur_client_library', 'start_ursim.sh', '-m', ur_type],
            output='screen'
        ),

        # Step 2: UR Control driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_robot_driver'),
                    'launch',
                    'ur_control.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': ur_type,
                'robot_ip': robot_ip,
                'launch_rviz': launch_rviz
            }.items()
        ),

        # Step 3: UR MoveIt configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_moveit_config'),
                    'launch',
                    'ur_moveit.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': ur_type,
                'launch_rviz': launch_rviz
            }.items()
        )
    ])
