#!/usr/bin/env python3
"""Live OpenVLA -> IK -> YAM arm bridge + RViz.

Starts ONLY the bridge node (see vla_bridge_node.py's module docstring for
the full pipeline and safety model) and, by default, RViz with a ghost
"next IK-solved step" display alongside the live arm. Prerequisites this
launch file does NOT start:

  - The arm's own bringup (robot_state_publisher, ros2_control, controllers,
    and move_group for /compute_ik) - e.g.
        ros2 launch i2rt_moveit_config demo.launch.py
    (defaults to mock hardware; add use_mock_hardware:=false only once
    confirmed safe to run on the real arm).
  - A camera publishing on image_topic (default /camera/camera/color/
    image_raw) - e.g. a realsense2_camera launch.
  - The OpenVLA inference server (openvla/vla-scripts/deploy.py), pointed at
    a futur_vla checkpoint directory.

The bridge starts DISARMED - see vla_bridge_node.py's module docstring for
how to arm it once you've confirmed the RViz ghost is tracking sane targets.

    ros2 launch i2rt_vla_bridge vla_bridge.launch.py \\
        instruction:="pick up the red block" server_url:=http://localhost:8000
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_ns_arg = DeclareLaunchArgument(
        "arm_ns",
        default_value="",
        description=(
            "ROS namespace the target arm's controllers/joint_states live under. Empty (default) matches "
            "i2rt_moveit_config/demo.launch.py's un-namespaced single-arm setup. NOTE: move_group's /compute_ik "
            "and TF are NOT namespace-aware in this repo today (see i2rt_teleop's dual-arm TF caveat) - setting "
            "this to 'follower'/'leader' only works if you've separately arranged a namespaced move_group and "
            "unambiguous TF for that arm."
        ),
    )
    server_url_arg = DeclareLaunchArgument(
        "server_url", default_value="http://localhost:8000", description="OpenVLA inference server base URL."
    )
    instruction_arg = DeclareLaunchArgument(
        "instruction", default_value="", description="Fixed language instruction sent with every /act request."
    )
    unnorm_key_arg = DeclareLaunchArgument(
        "unnorm_key", default_value="yam_packaging", description="dataset_statistics.json key for the loaded checkpoint."
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic", default_value="/camera/camera/color/image_raw", description="Camera color image topic."
    )
    control_rate_hz_arg = DeclareLaunchArgument(
        "control_rate_hz",
        default_value="5.0",
        description=(
            "Bridge control-loop rate; matches training resample_hz. A tick that takes longer than "
            "1/control_rate_hz (e.g. slow inference) just runs at whatever slower rate results - it does not "
            "pile up or overlap."
        ),
    )
    request_timeout_s_arg = DeclareLaunchArgument(
        "request_timeout_s",
        default_value="10.0",
        description=(
            "HTTP read timeout (seconds) for each OpenVLA /act call. A 7B model's autoregressive decode on a "
            "single GPU commonly takes 1-3+ seconds (more on the first call, while CUDA warms up) - raise this "
            "if you still see 'Read timed out' warnings with a 200 OK logged late on the server side."
        ),
    )
    ik_timeout_s_arg = DeclareLaunchArgument(
        "ik_timeout_s",
        default_value="0.2",
        description=(
            "Timeout (seconds) passed to each /compute_ik call. KDL's numeric solver needs real iteration budget, "
            "especially from a near-singular seed (e.g. all-zero joints on mock-hardware startup) - raise this "
            "further if you still see IK failures (error_code=-31, NO_IK_SOLUTION)."
        ),
    )
    max_dxyz_m_arg = DeclareLaunchArgument(
        "max_dxyz_m", default_value="0.03", description="Per-tick translation delta clamp, meters."
    )
    max_drot_deg_arg = DeclareLaunchArgument(
        "max_drot_deg", default_value="15.0", description="Per-tick rotation delta clamp, degrees."
    )
    max_joint_velocity_arg = DeclareLaunchArgument(
        "max_joint_velocity",
        default_value="0.3",
        description=(
            "rad/s slew-rate cap applied to every VLA-driven joint target (same technique as "
            "i2rt_teleop's leader_follower_node.cpp) - bounds real-world arm speed independent of "
            "MoveIt's own joint_limits.yaml scaling, which does NOT apply to this node (it only "
            "calls /compute_ik, never move_group's plan/execute pipeline). Lower this if you still "
            "see overshoot; per-joint overrides are a node parameter, not a launch arg - array "
            "launch args are awkward via CLI, override directly with "
            "--ros-args -p max_joint_velocity_per_joint:='[...]' if needed."
        ),
    )
    gripper_closed_pos_arg = DeclareLaunchArgument("gripper_closed_pos", default_value="0.0")
    gripper_open_pos_arg = DeclareLaunchArgument("gripper_open_pos", default_value="-0.0475")
    home_ramp_enabled_arg = DeclareLaunchArgument(
        "home_ramp_enabled",
        default_value="true",
        choices=["true", "false"],
        description=(
            "true (default): once armed, ramp to home_joint_positions and hold before VLA control begins. "
            "false: skip the ramp entirely and begin VLA-driven control immediately from wherever the arm "
            "currently is - for debugging model output from a manually-positioned pose (e.g. via "
            "i2rt_teleop's command_pose_node)."
        ),
    )
    home_ramp_duration_s_arg = DeclareLaunchArgument(
        "home_ramp_duration_s",
        default_value="4.0",
        description=(
            "Once armed, seconds to smoothly ramp the arm from its current pose to home_joint_positions "
            "(a node parameter, not a launch arg - array launch args are awkward via CLI; override it directly "
            "with --ros-args -p home_joint_positions:='[...]' if the default debug pose doesn't fit your setup) "
            "before VLA-driven control begins."
        ),
    )
    home_hold_duration_s_arg = DeclareLaunchArgument(
        "home_hold_duration_s", default_value="1.0", description="Seconds to hold at home_joint_positions after ramping, before VLA control begins."
    )
    home_max_joint_velocity_arg = DeclareLaunchArgument(
        "home_max_joint_velocity", default_value="1.0", description="rad/s clamp on the home ramp - extends home_ramp_duration_s if needed, never shortens it."
    )
    crop_width_arg = DeclareLaunchArgument("crop_width", default_value="960")
    image_size_arg = DeclareLaunchArgument("image_size", default_value="224")
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz", default_value="true", choices=["true", "false"], description="Also start RViz with the ghost display."
    )

    vla_bridge_node = Node(
        package="i2rt_vla_bridge",
        executable="vla_bridge_node",
        namespace=LaunchConfiguration("arm_ns"),
        output="screen",
        parameters=[
            {
                "server_url": LaunchConfiguration("server_url"),
                "instruction": LaunchConfiguration("instruction"),
                "unnorm_key": LaunchConfiguration("unnorm_key"),
                "image_topic": LaunchConfiguration("image_topic"),
                "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                "request_timeout_s": LaunchConfiguration("request_timeout_s"),
                "ik_timeout_s": LaunchConfiguration("ik_timeout_s"),
                "max_dxyz_m": LaunchConfiguration("max_dxyz_m"),
                "max_drot_deg": LaunchConfiguration("max_drot_deg"),
                "max_joint_velocity": LaunchConfiguration("max_joint_velocity"),
                "gripper_closed_pos": LaunchConfiguration("gripper_closed_pos"),
                "gripper_open_pos": LaunchConfiguration("gripper_open_pos"),
                "home_ramp_enabled": LaunchConfiguration("home_ramp_enabled"),
                "home_ramp_duration_s": LaunchConfiguration("home_ramp_duration_s"),
                "home_hold_duration_s": LaunchConfiguration("home_hold_duration_s"),
                "home_max_joint_velocity": LaunchConfiguration("home_max_joint_velocity"),
                "crop_width": LaunchConfiguration("crop_width"),
                "image_size": LaunchConfiguration("image_size"),
            }
        ],
    )

    rviz_config = os.path.join(get_package_share_directory("i2rt_vla_bridge"), "config", "vla_inference.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(
        [
            arm_ns_arg,
            server_url_arg,
            instruction_arg,
            unnorm_key_arg,
            image_topic_arg,
            control_rate_hz_arg,
            request_timeout_s_arg,
            ik_timeout_s_arg,
            max_dxyz_m_arg,
            max_drot_deg_arg,
            max_joint_velocity_arg,
            gripper_closed_pos_arg,
            gripper_open_pos_arg,
            home_ramp_enabled_arg,
            home_ramp_duration_s_arg,
            home_hold_duration_s_arg,
            home_max_joint_velocity_arg,
            crop_width_arg,
            image_size_arg,
            launch_rviz_arg,
            vla_bridge_node,
            rviz_node,
        ]
    )
