from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo
)
from launch.event_handlers import OnProcessExit
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
            default_value='127.0.0.1',
            description='IP address of the robot/simulator'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'headless_mode',
            default_value='true',
            description='Enable headless mode for robot control'
        ),
        DeclareLaunchArgument(
            'controller_spawner_timeout',
            default_value='60',
            description='Timeout used when spawning controllers.'
        ),

        # Step 1: Start URSim in detached mode
        ExecuteProcess(
            cmd=['ros2', 'run', 'ur_client_library', 'start_ursim.sh', '-m', ur_type, '-d'],
            output='screen'
        ),

        # Step 1.1: Wait for simulator ports to be ready
        wait_for_sim = ExecuteProcess(
            cmd=['python3', '-c',
                 'import socket, time; '
                 'host, ports = "127.0.0.1", [30001, 30002, 30003, 29999]; '
                 'start = time.time(); '
                 'print(f"Waiting for URSim on {host} ports {ports}..."); '
                 'while time.time() - start < 120: '
                 '    ready = True; '
                 '    for port in ports: '
                 '        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s: '
                 '            if s.connect_ex((host, port)) != 0: '
                 '                ready = False; break; '
                 '    if ready: '
                 '        print("All ports are open. Waiting 5s for stabilization..."); '
                 '        time.sleep(5); '
                 '        print("URSim is ready!"); exit(0); '
                 '    time.sleep(1); '
                 'print("Timeout waiting for URSim"); exit(1)'],
            output='screen'
        ),

        # Step 2: UR Control driver
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
                'robot_ip': robot_ip,
                'launch_rviz': launch_rviz,
                'headless_mode': LaunchConfiguration('headless_mode'),
                'controller_spawner_timeout': LaunchConfiguration('controller_spawner_timeout'),
            }.items()
        ),

        # Step 3: UR MoveIt configuration
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
                'launch_rviz': launch_rviz
            }.items()
        ),

        # Chain the execution: wait for sim, then start driver and moveit
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_sim,
                on_exit=[
                    LogInfo(msg='URSim is ready, starting driver and moveit...'),
                    driver_launch,
                    moveit_launch
                ],
            )
        ),

        wait_for_sim
    ])
