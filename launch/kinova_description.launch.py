import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


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


def make_arm_only_robot_description():
    moveit_share = get_package_share_directory("kinova_gen3_lite_moveit_config")
    xacro_path = os.path.join(moveit_share, "config", "gen3_lite.urdf.xacro")
    initial_positions_path = os.path.join(moveit_share, "config", "initial_positions.yaml")

    robot_doc = xacro.process_file(
        xacro_path,
        mappings={"initial_positions_file": initial_positions_path},
    )
    root = ET.fromstring(robot_doc.toxml())

    for element in list(root):
        name = element.get("name")
        if element.tag == "link" and is_gripper_name(name):
            root.remove(element)
        elif element.tag == "joint" and joint_references_gripper(element):
            root.remove(element)
        elif element.tag in ("gazebo", "ros2_control"):
            root.remove(element)

    return ET.tostring(root, encoding="unicode")


def write_robot_description_file(robot_description):
    urdf_path = os.path.join("/tmp", "slicer_ros2_kinova_gen3_lite_arm_only.urdf")
    with open(urdf_path, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(robot_description)
    return urdf_path


def launch_setup(context, *args, **kwargs):
    robot_description = make_arm_only_robot_description()
    urdf_path = write_robot_description_file(robot_description)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        arguments=[urdf_path],
        output="screen",
    )

    return [robot_state_publisher, joint_state_publisher_gui]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
