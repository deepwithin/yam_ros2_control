#!/usr/bin/env python3
"""MoveIt 2 for YAM-family arms. Defaults to mock hardware — no CAN, no arm
needed — for planning/IK/URDF checks.

Brings up robot_state_publisher, controller_manager (mock_components/
GenericSystem by default, or the real i2rt_hardware_interface/
YamSystemInterface if use_mock_hardware:=false), joint_state_broadcaster +
joint_trajectory_controller, move_group, and RViz with the MotionPlanning
panel pre-loaded.

    ros2 launch i2rt_moveit_config demo.launch.py robot:=big_yam_linear_4310

To drive the REAL arm — only after independently confirming activation holds
position cleanly with no MoveIt/trajectory controller involved (see
i2rt_bringup's yam_ros2_control.launch.py) — add:

    use_mock_hardware:=false can_channel:=can0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        "robot",
        description="Which robot variant's URDF to load. One of: yam_crank_4310, big_yam_linear_4310.",
        choices=["yam_crank_4310", "big_yam_linear_4310"],
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description=(
            "true (default): ros2_control's built-in simulator, no CAN/arm needed. "
            "false: the real i2rt_hardware_interface plugin — only after confirming "
            "activation holds position cleanly with plain ros2_control first, no MoveIt "
            "involved. See i2rt_bringup/yam_ros2_control.launch.py."
        ),
        choices=["true", "false"],
    )
    can_channel_arg = DeclareLaunchArgument(
        "can_channel",
        default_value="can0",
        description="SocketCAN interface the arm is on. Ignored when use_mock_hardware:=true.",
    )

    urdf_file = PathJoinSubstitution(
        [FindPackageShare("i2rt_description"), "urdf", [LaunchConfiguration("robot"), ".urdf.xacro"]]
    )
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                urdf_file,
                " use_mock_hardware:=",
                LaunchConfiguration("use_mock_hardware"),
                " can_channel:=",
                LaunchConfiguration("can_channel"),
            ]
        ),
        value_type=str,
    )

    moveit_config_share = get_package_share_directory("i2rt_moveit_config")
    with open(os.path.join(moveit_config_share, "config", "i2rt.srdf")) as f:
        robot_description_semantic = f.read()

    kinematics_yaml = os.path.join(moveit_config_share, "config", "kinematics.yaml")
    joint_limits_yaml = os.path.join(moveit_config_share, "config", "joint_limits.yaml")
    ompl_yaml = os.path.join(moveit_config_share, "config", "ompl_planning.yaml")
    moveit_controllers_yaml = os.path.join(moveit_config_share, "config", "moveit_controllers.yaml")
    rviz_config = os.path.join(moveit_config_share, "config", "moveit.rviz")

    ros2_controllers_yaml = os.path.join(
        get_package_share_directory("i2rt_description"), "config", "yam_controllers.yaml"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, ros2_controllers_yaml],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            kinematics_yaml,
            joint_limits_yaml,
            ompl_yaml,
            moveit_controllers_yaml,
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
            # MoveIt's default (0.01 rad) rejects execution if the real arm's
            # current position has drifted this much from the planned
            # trajectory's start point by the time Execute is pressed. The
            # real arm has no gravity compensation yet (see
            # i2rt_hardware_interface — the Python reference's KDL-based
            # inverse-dynamics feedforward was never ported), so it can sag
            # measurably while just holding. Raised as a stopgap; the real
            # fix is gravity compensation, not a looser tolerance.
            {"trajectory_execution.allowed_start_tolerance": 0.05},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            kinematics_yaml,
            ompl_yaml,
        ],
    )

    return LaunchDescription(
        [
            robot_arg,
            use_mock_hardware_arg,
            can_channel_arg,
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            move_group_node,
            rviz_node,
        ]
    )
