import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


GRIPPER_NAME_PARTS = ("gripper", "finger")


def is_gripper_name(name):
    return name is not None and any(part in name for part in GRIPPER_NAME_PARTS)


def joint_references_gripper(joint):
    parent = joint.find("parent")
    child = joint.find("child")
    return (
        is_gripper_name(joint.get("name"))
        or (parent is not None and is_gripper_name(parent.get("link")))
        or (child is not None and is_gripper_name(child.get("link")))
    )


def remove_gripper_from_robot_description(root):
    for element in list(root):
        name = element.get("name")
        if element.tag == "link" and is_gripper_name(name):
            root.remove(element)
        elif element.tag == "joint" and joint_references_gripper(element):
            root.remove(element)
        elif element.tag == "gazebo" and is_gripper_name(element.get("reference")):
            root.remove(element)

    for control in root.findall("ros2_control"):
        for joint in list(control.findall("joint")):
            if is_gripper_name(joint.get("name")):
                control.remove(joint)


def make_arm_only_semantic_description():
    moveit_share = get_package_share_directory("kinova_gen3_lite_moveit_config")
    srdf_path = os.path.join(moveit_share, "config", "gen3_lite.srdf")
    root = ET.parse(srdf_path).getroot()

    for element in list(root):
        if element.tag == "group" and element.get("name") == "gripper":
            root.remove(element)
        elif element.tag == "passive_joint" and is_gripper_name(element.get("name")):
            root.remove(element)
        elif element.tag == "disable_collisions" and (
            is_gripper_name(element.get("link1")) or is_gripper_name(element.get("link2"))
        ):
            root.remove(element)

    return ET.tostring(root, encoding="unicode")


def remove_gripper_controller(moveit_parameters):
    controllers = moveit_parameters.get("moveit_simple_controller_manager")
    if controllers is None:
        return

    controller_names = controllers.get("controller_names", [])
    controllers["controller_names"] = [
        name for name in controller_names if name != "gen3_lite_2f_gripper_controller"
    ]
    controllers.pop("gen3_lite_2f_gripper_controller", None)


def remove_gripper_joint_limits(moveit_parameters):
    planning = moveit_parameters.get("robot_description_planning", {})
    joint_limits = planning.get("joint_limits", {})
    for joint_name in list(joint_limits):
        if is_gripper_name(joint_name):
            joint_limits.pop(joint_name)


def make_fake_hardware_robot_description():
    moveit_share = get_package_share_directory("kinova_gen3_lite_moveit_config")
    xacro_path = os.path.join(moveit_share, "config", "gen3_lite.urdf.xacro")
    initial_positions_path = os.path.join(moveit_share, "config", "initial_positions.yaml")

    robot_doc = xacro.process_file(
        xacro_path,
        mappings={"initial_positions_file": initial_positions_path},
    )
    root = ET.fromstring(robot_doc.toxml())
    root.set("name", "gen3_lite_gen3_lite_2f")

    # The generated Kinova URDF includes its real hardware interface. For this
    # quick simulation launch, keep only the FakeSystem ros2_control block.
    for control in list(root.findall("ros2_control")):
        if control.get("name") == "KortexMultiInterfaceHardware":
            root.remove(control)
    remove_gripper_from_robot_description(root)

    return ET.tostring(root, encoding="unicode")


def make_moveit_parameters(robot_description):
    moveit_share = get_package_share_directory("kinova_gen3_lite_moveit_config")
    initial_positions_path = os.path.join(moveit_share, "config", "initial_positions.yaml")

    moveit_config = (
        MoveItConfigsBuilder("gen3_lite", package_name="kinova_gen3_lite_moveit_config")
        .robot_description(
            file_path="config/gen3_lite.urdf.xacro",
            mappings={"initial_positions_file": initial_positions_path},
        )
        .robot_description_semantic(file_path="config/gen3_lite.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    moveit_parameters = moveit_config.to_dict()
    moveit_parameters["robot_description"] = robot_description
    moveit_parameters["robot_description_semantic"] = make_arm_only_semantic_description()
    remove_gripper_controller(moveit_parameters)
    remove_gripper_joint_limits(moveit_parameters)
    return moveit_parameters


def launch_setup(context, *args, **kwargs):
    moveit_share = get_package_share_directory("kinova_gen3_lite_moveit_config")
    robot_description = make_fake_hardware_robot_description()
    moveit_parameters = make_moveit_parameters(robot_description)
    ros2_controllers_path = os.path.join(moveit_share, "config", "ros2_controllers.yaml")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="both",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            ros2_controllers_path,
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_parameters],
        output="screen",
    )

    return [
        robot_state_publisher,
        ros2_control_node,
        TimerAction(
            period=2.0,
            actions=[
                joint_state_broadcaster_spawner,
                joint_trajectory_controller_spawner,
            ],
        ),
        move_group,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
