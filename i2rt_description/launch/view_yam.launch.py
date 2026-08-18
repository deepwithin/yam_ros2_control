#!/usr/bin/env python3
"""
Visualize the big_yam + linear_4310 description in RViz.

Brings up robot_state_publisher (fed by big_yam_linear_4310.urdf.xacro) plus
joint_state_publisher_gui for manual slider control, and RViz. No
controller_manager is started here, so the <ros2_control> block in the xacro
is inert -- the CAN channel is never opened, regardless of can_channel/
use_mock_hardware (see big_yam_linear_4310.urdf.xacro's comment). Those args
only matter once this description is consumed by a real control launch file.

RViz defaults to i2rt_description/rviz/view_robot.rviz (RobotModel sourced
from the /robot_description topic + TF + Grid, Fixed Frame "world" -- matches
this xacro's root link). Pass rviz_config:="" to fall back to RViz's default
unconfigured view instead, or point it at a different config entirely.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_channel = LaunchConfiguration("can_channel")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    declared_arguments = [
        DeclareLaunchArgument(
            "can_channel",
            default_value="can0",
            description="CAN interface name forwarded into the xacro's ros2_control block.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description=(
                "Forwarded into the xacro's ros2_control block. Irrelevant here since no "
                "controller_manager is started by this launch file, but kept true by default "
                "so this file never implies a real CAN connection."
            ),
        ),
        DeclareLaunchArgument(
            "use_joint_state_publisher_gui",
            default_value="true",
            description="Launch joint_state_publisher_gui for manual joint sliders.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("i2rt_description"), "rviz", "view_robot.rviz"]
            ),
            description=(
                "Path to an RViz config file. Pass an empty string to launch RViz with its "
                "default (unsaved, no displays configured) view instead."
            ),
        ),
    ]

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("i2rt_description"), "urdf", "big_yam_linear_4310.urdf.xacro"]
    )

    robot_description_content = Command(
        [
            "xacro ",
            xacro_file,
            " can_channel:=",
            can_channel,
            " use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(use_joint_state_publisher_gui),
    )

    rviz_node_no_config = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        condition=IfCondition(
            PythonExpression(["'", use_rviz, "' == 'true' and '", rviz_config, "' == ''"])
        ),
    )

    rviz_node_with_config = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(
            PythonExpression(["'", use_rviz, "' == 'true' and '", rviz_config, "' != ''"])
        ),
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node_no_config,
            rviz_node_with_config,
        ]
    )
