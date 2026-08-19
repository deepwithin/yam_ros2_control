#!/usr/bin/env python3
"""Leader-follower bringup for two YAM arms: one ros2_control stack per arm
under its own namespace ('leader'/'follower'; each still uses unprefixed
joint1..joint6 internally, disambiguated only by namespace - see
leader_follower_node.cpp's class comment for why no cross-arm zero-alignment
is assumed or checked automatically) plus the mirroring node itself.

The leader arm is hand-backdrivable by default (compliant_mode - see
big_yam_ros2_control.xacro's macro comment): gravity-comp-only stiffness,
no position controller spawned at all, since it's meant to be hand-guided
and only read from, never commanded. The follower keeps its normal stiff
position-hold behavior and full controller set, since it's the one being
driven.

Defaults to mock hardware on both sides for safe, no-CAN-needed testing
(mock hardware has no impedance/MIT control model, so compliant_mode has no
real effect there - the follower's own position tracking is still visible,
just not the leader's backdrivability):

    ros2 launch i2rt_teleop leader_follower.launch.py

Real hardware (only after independently confirming both arms hold position
cleanly with plain ros2_control first, no teleop node involved, AND manually
verifying leader/follower joint-zero alignment - see leader_follower_node.cpp):

    ros2 launch i2rt_teleop leader_follower.launch.py use_mock_hardware:=false \\
        leader_can_channel:=can0 follower_can_channel:=can1 alignment_confirmed:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def arm_nodes(namespace, can_channel, compliant_mode, spawn_position_controllers):
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("i2rt_description"), "urdf", "big_yam_linear_4310.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                urdf_file,
                " use_mock_hardware:=",
                LaunchConfiguration("use_mock_hardware"),
                " can_channel:=",
                can_channel,
                " compliant_mode:=",
                compliant_mode,
                " compliant_kp:=",
                LaunchConfiguration("compliant_kp"),
                " compliant_kd:=",
                LaunchConfiguration("compliant_kd"),
            ]
        ),
        value_type=str,
    )
    ros2_controllers_yaml = os.path.join(
        get_package_share_directory("i2rt_description"), "config", "yam_controllers.yaml"
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=namespace,
            parameters=[{"robot_description": robot_description}, ros2_controllers_yaml],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["joint_state_broadcaster"],
        ),
    ]
    # A compliant/hand-guided arm (the leader) has nothing for these to
    # usefully do - joint_trajectory_controller would hold whatever position
    # it was in at spawn time via the "position" command interface, which is
    # harmless once compliant_mode has already zeroed kp (the hold becomes a
    # no-op), but there's no reason to run a position controller on an arm
    # that's never meant to be commanded, and it removes a footgun if
    # compliant_mode is ever misconfigured.
    if spawn_position_controllers:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=["joint_trajectory_controller"],
            )
        )
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=["gripper_controller"],
            )
        )
    return nodes


def generate_launch_description():
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description=(
            "true (default): ros2_control's built-in simulator for both arms, no CAN/hardware needed. "
            "false: real i2rt_hardware_interface on both sides - only after independently confirming each arm "
            "holds position cleanly with plain ros2_control, and manually verifying leader/follower joint-zero "
            "alignment (see leader_follower_node.cpp's class comment)."
        ),
        choices=["true", "false"],
    )
    leader_can_channel_arg = DeclareLaunchArgument(
        "leader_can_channel", default_value="can0", description="Leader arm's SocketCAN interface."
    )
    follower_can_channel_arg = DeclareLaunchArgument(
        "follower_can_channel", default_value="can1", description="Follower arm's SocketCAN interface."
    )
    leader_compliant_mode_arg = DeclareLaunchArgument(
        "leader_compliant_mode",
        default_value="true",
        description=(
            "true (default): leader arm is hand-backdrivable (gravity-comp-only stiffness, no position "
            "controller spawned) - see big_yam_ros2_control.xacro's macro comment. Real hardware only; "
            "mock hardware has no impedance model so this has no effect there."
        ),
        choices=["true", "false"],
    )
    compliant_kp_arg = DeclareLaunchArgument(
        "compliant_kp",
        default_value="0.0",
        description=(
            "Gentle anti-drift position gain on the compliant arm(s) when compliant_mode is true - see "
            "YamSystemInterface's class comment. Defaults to 0.0 (pure gravity-comp, no drift correction) so "
            "enabling compliant_mode never changes hardware behavior until you explicitly raise this; tune up "
            "incrementally if the arm drifts out of place when left alone."
        ),
    )
    compliant_kd_arg = DeclareLaunchArgument(
        "compliant_kd",
        default_value="0.3",
        description="Viscous damping applied on the compliant arm(s) when compliant_mode is true.",
    )
    alignment_confirmed_arg = DeclareLaunchArgument(
        "alignment_confirmed",
        default_value="false",
        description=(
            "Must be manually set true after physically verifying the leader and follower agree on what joint "
            "position 0 means (command both to the same known pose independently and visually confirm they "
            "match) - see leader_follower_node.cpp's class comment. The node stays inert until this is true."
        ),
        choices=["true", "false"],
    )
    max_joint_velocity_arg = DeclareLaunchArgument(
        "max_joint_velocity",
        default_value="1.0",
        description="rad/s clamp applied to every commanded step, including the startup ramp's minimum duration.",
    )

    leader_follower_node = Node(
        package="i2rt_teleop",
        executable="leader_follower_node",
        parameters=[
            {
                "alignment_confirmed": LaunchConfiguration("alignment_confirmed"),
                "max_joint_velocity": LaunchConfiguration("max_joint_velocity"),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            use_mock_hardware_arg,
            leader_can_channel_arg,
            follower_can_channel_arg,
            leader_compliant_mode_arg,
            compliant_kp_arg,
            compliant_kd_arg,
            alignment_confirmed_arg,
            max_joint_velocity_arg,
            *arm_nodes(
                "leader",
                LaunchConfiguration("leader_can_channel"),
                LaunchConfiguration("leader_compliant_mode"),
                spawn_position_controllers=False,
            ),
            *arm_nodes(
                "follower",
                LaunchConfiguration("follower_can_channel"),
                "false",
                spawn_position_controllers=True,
            ),
            leader_follower_node,
        ]
    )
