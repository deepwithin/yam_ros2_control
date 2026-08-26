#!/usr/bin/env python3
"""Live OpenVLA -> IK -> YAM arm bridge.

Per control tick (control_rate_hz, default 5.0 Hz - matches the training-
time resample rate in futur_vla/scripts/bag_to_npy.py, which used no action
chunking):

  1. Look up the current base_link->gripper_tip pose (TF) and the current
     link_6->gripper_tip offset (TF; reflects the current gripper opening).
  2. POST the latest camera frame + a fixed instruction to the OpenVLA
     server's /act endpoint (see openvla/vla-scripts/deploy.py); receive back
     a 7-dim action [dx, dy, dz, droll, dpitch, dyaw, gripper_abs] expressed
     in the gripper_tip frame (bag_to_npy.py --ee_link gripper_tip).
  3. Clamp the delta's magnitude (safety net against a bad model output),
     then integrate it onto the current gripper_tip pose (pose_math.
     integrate_delta - same rotation-composition order used when the deltas
     were generated at data-collection time).
  4. Back-solve the equivalent target pose for link_6 (pose_math.
     backsolve_parent_target), because the "arm" MoveIt group's IK chain
     tips at link_6, not gripper_tip - gripper_tip sits past the gripper's
     own *passive* prismatic joint, which must stay decoupled from arm IK.
  5. Call the already-configured KDL IK solver via move_group's
     /compute_ik service (moveit_msgs/srv/GetPositionIK) for group "arm".
  6. Publish a moveit_msgs/DisplayRobotState ghost of the solved joint
     target (for RViz) - this step always runs, even before the node is
     armed, so the ghost is visible for preview.
  7. If armed: stream the solution to joint_trajectory_controller (single-
     point JointTrajectory, fresh each tick, matching i2rt_teleop's
     leader_follower_node.cpp streaming style) and the gripper's absolute
     command to gripper_controller's GripperCommand action.

SAFETY MODEL:

  - The node starts DISARMED (enabled=False). While disarmed it still runs
    the full pipeline above and publishes the RViz ghost + target pose, but
    never publishes to joint_trajectory_controller or gripper_controller.
    Arm it with:
        ros2 service call /vla_bridge_node/enable std_srvs/srv/SetBool "{data: true}"
    (mirrors the alignment_confirmed gate in i2rt_teleop's
    leader_follower_node.cpp - inspect the ghost tracking sane targets
    before arming, exactly like confirming leader/follower alignment there.)
  - Arming does NOT immediately hand control to the VLA output. The very
    first thing that happens once armed is a smooth ramp of the real arm
    from its current joint state to home_joint_positions (a known-good,
    non-singular pose), followed by a hold (home_ramp_duration_s /
    home_hold_duration_s) - only once that completes does VLA-driven
    joint_trajectory/gripper_controller commanding actually begin. This
    avoids ever starting VLA control from whatever arbitrary (and often
    near-singular - see _solve_ik's diagnostics) pose the arm happened to
    be in when armed. Set home_ramp_enabled:=false to skip this and begin
    VLA-driven control immediately from wherever the arm currently is - for
    debugging model output starting from a specific, manually-driven pose
    (e.g. via i2rt_teleop's command_pose_node) instead of home_joint_positions.
  - Every delta's translation/rotation magnitude is clamped
    (max_dxyz_m / max_drot_deg) before being integrated, independent of
    whatever the model itself outputs. This is a CARTESIAN clamp on the
    pre-IK step, not a joint-space one - a small Cartesian step taken near a
    kinematic singularity (see _solve_ik's diagnostics) can still IK-solve to
    a large joint-space jump.
  - Independently, every VLA-driven joint target is slew-limited to
    max_joint_velocity (rad/s, optionally overridden per joint via
    max_joint_velocity_per_joint) before being published, exactly like
    i2rt_teleop's leader_follower_node.cpp mirror_tick(): each joint is
    clamped to within max_joint_velocity/control_rate_hz of the last
    COMMANDED position (not the last measured one, for the same
    interpolate_from_desired_state reasoning as the teleop node). This is
    what actually bounds real-world arm speed for VLA-driven motion - MoveIt's
    own joint_limits.yaml velocity scaling (see i2rt_moveit_config) does NOT
    apply here, since this node never goes through move_group's plan/execute
    pipeline, only its /compute_ik service.
  - A flaky OpenVLA server call, a TF lookup failure, or an IK failure all
    result in "skip this tick, hold the last commanded state" - never a
    crash and never a partial/malformed command.
  - IK is always solved fresh from the arm's live current joint state (used
    as the seed), not from an internally-integrated joint target - so a
    stall or a skipped tick can't drift the seed away from reality.
"""
import hashlib
import time
from math import copysign, radians

import cv2
import numpy as np
import rclpy
from control_msgs.action import GripperCommand
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayRobotState, MoveItErrorCodes, ObjectColor
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from i2rt_vla_bridge import pose_math

try:
    import json_numpy

    json_numpy.patch()  # lets `requests` transparently encode/decode numpy arrays in JSON, both directions.
except ImportError as exc:  # pragma: no cover - startup-time dependency check
    raise ImportError(
        "json_numpy is required (pip install json-numpy) to talk to the OpenVLA server's /act endpoint - "
        "see openvla/vla-scripts/deploy.py's module docstring for the wire format this matches."
    ) from exc
import requests


class VlaBridgeNode(Node):
    def __init__(self):
        super().__init__("vla_bridge_node")

        self._server_url = self.declare_parameter("server_url", "http://localhost:8000").value
        self.declare_parameter("instruction", "")
        self._unnorm_key = self.declare_parameter("unnorm_key", "yam_packaging").value
        image_topic = self.declare_parameter("image_topic", "/camera/camera/color/image_raw").value
        compute_ik_service = self.declare_parameter("compute_ik_service", "/compute_ik").value
        self._ik_group_name = self.declare_parameter("ik_group_name", "arm").value
        self._ik_link_name = self.declare_parameter("ik_link_name", "link_6").value
        self._base_link = self.declare_parameter("base_link", "base_link").value
        self._ee_link = self.declare_parameter("ee_link", "gripper_tip").value
        self._arm_joint_names = self.declare_parameter(
            "arm_joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        ).value
        self._gripper_joint_name = self.declare_parameter("gripper_joint_name", "gripper_joint").value
        # Links whose ghost highlight color should be set. Joint names above
        # drive the ghost's joint_state; this drives which *links* light up -
        # the gripper's own child links move with gripper_joint even though
        # it isn't one of arm_joint_names above.
        self._ghost_link_names = self.declare_parameter(
            "ghost_link_names",
            ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "tip_left", "tip_right", "gripper_tip"],
        ).value

        self._control_rate_hz = self.declare_parameter("control_rate_hz", 0.4).value
        # KDL's numeric (LMA) solver needs real iteration budget, especially
        # from a seed at or near a singular configuration (e.g. all-zero
        # joints, common on mock-hardware startup) - 30ms starved it in
        # practice (NO_IK_SOLUTION, error_code -31, on nearly every tick).
        # The control loop is bottlenecked by VLA inference latency (seconds)
        # regardless, so there's no real cost to giving IK much more room.
        self._ik_timeout_s = self.declare_parameter("ik_timeout_s", 0.2).value
        # A 7B VLA's autoregressive decode on a single GPU commonly takes
        # 1-3+ seconds per call (more on the very first request, while CUDA
        # kernels/JIT warm up) - 0.5s is nowhere near enough and will make
        # every single tick "fail" even though the server eventually answers
        # with 200 OK. Raise this (via the request_timeout_s launch arg) to
        # comfortably exceed your server's actual observed /act latency;
        # control_rate_hz just governs how often we'd LIKE to tick - a tick
        # that takes longer than 1/control_rate_hz simply runs at whatever
        # slower rate the model allows, it does not pile up or overlap
        # (the timer callback's own group is mutually exclusive with itself).
        self._request_timeout_s = self.declare_parameter("request_timeout_s", 10.0).value
        self._max_dxyz_m = self.declare_parameter("max_dxyz_m", 0.03).value
        self._max_drot_rad = radians(self.declare_parameter("max_drot_deg", 15.0).value)
        # Raw gripper_joint readings (meters) at fully closed/open - must
        # match gripper_linear_4310.xacro's <limit> tag and the same
        # constants futur_vla/scripts/bag_to_npy.py used to build the
        # training data's normalized [0,1] gripper channel.
        self._gripper_closed_pos = self.declare_parameter("gripper_closed_pos", 0.0).value
        self._gripper_open_pos = self.declare_parameter("gripper_open_pos", -0.0475).value
        self._gripper_position_threshold = self.declare_parameter("gripper_position_threshold", 0.002).value
        self._gripper_max_effort = self.declare_parameter("gripper_max_effort", 0.0).value
        self._crop_width = self.declare_parameter("crop_width", 960).value
        self._image_size = self.declare_parameter("image_size", 224).value
        self._trajectory_time_from_start_s = self.declare_parameter("trajectory_time_from_start_s", 0.15).value
        # rad/s clamp applied to every VLA-driven joint target before it's
        # published (see SAFETY MODEL above) - same slew-limiting technique
        # as i2rt_teleop's leader_follower_node.cpp mirror_tick(). Defaults
        # well below home_max_joint_velocity: VLA output is a trained
        # checkpoint's raw guess, not a human's deliberate motion, and large
        # per-tick deltas (control_rate_hz is inference-bound, often <1 Hz)
        # combined with IK's nonlinearity near singularities can otherwise
        # produce fast, overshoot-then-correct motion. Lower this further if
        # you still see overshoot; raise it once you trust a given checkpoint's
        # output to move faster.
        self._max_joint_velocity = self.declare_parameter("max_joint_velocity", 0.3).value
        # Optional per-joint override, same length/order as arm_joint_names.
        # Empty (default) = use max_joint_velocity for every joint.
        self._max_joint_velocity_per_joint = self.declare_parameter("max_joint_velocity_per_joint", []).value

        # Home ramp: once armed, before any VLA-driven command goes out, the
        # arm is smoothly ramped from its current joint state to a known-
        # good, non-singular pose and held there - avoids ever starting VLA
        # control from mock/real hardware's typical all-zero-joints startup
        # state, which is often a kinematic singularity (see the IK-failure
        # diagnostics in _solve_ik). Defaults to a pose the user has
        # previously used for exactly this purpose
        # (i2rt_teleop/src/command_pose_node.cpp).
        self._home_ramp_enabled = self.declare_parameter("home_ramp_enabled", True).value
        self._home_joint_positions = self.declare_parameter(
            "home_joint_positions",
            [0.37251087205310185, 0.35839627679865593, 0.6372549019607838, 0.5579079880979645, 0.0516899366750625, -0.04367895017929335],
        ).value
        self._home_ramp_duration_s = self.declare_parameter("home_ramp_duration_s", 4.0).value
        self._home_ramp_waypoints = self.declare_parameter("home_ramp_waypoints", 40).value
        self._home_hold_duration_s = self.declare_parameter("home_hold_duration_s", 1.0).value
        # rad/s clamp - if the gap between current and home is large, the
        # ramp duration is extended (never shortened) so no joint exceeds
        # this, same reasoning as i2rt_teleop's leader_follower_node.cpp.
        self._home_max_joint_velocity = self.declare_parameter("home_max_joint_velocity", 1.0).value

        joint_states_topic = self.declare_parameter("joint_states_topic", "joint_states").value
        joint_trajectory_topic = self.declare_parameter(
            "joint_trajectory_topic", "joint_trajectory_controller/joint_trajectory"
        ).value
        gripper_action_name = self.declare_parameter("gripper_action_name", "gripper_controller/gripper_cmd").value
        target_robot_state_topic = self.declare_parameter(
            "target_robot_state_topic", "vla_bridge/target_robot_state"
        ).value
        target_pose_topic = self.declare_parameter("target_pose_topic", "vla_bridge/target_pose").value
        debug_image_topic = self.declare_parameter("debug_image_topic", "vla_bridge/debug_image").value

        if self._control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be > 0")
        if self._gripper_open_pos == self._gripper_closed_pos:
            raise ValueError("gripper_open_pos and gripper_closed_pos must differ")
        if len(self._home_joint_positions) != len(self._arm_joint_names):
            raise ValueError("home_joint_positions must be the same length as arm_joint_names")
        if self._home_max_joint_velocity <= 0.0 or self._home_ramp_waypoints < 1 or self._home_ramp_duration_s < 0.0:
            raise ValueError("home_max_joint_velocity must be > 0, home_ramp_waypoints >= 1, home_ramp_duration_s >= 0")
        if self._max_joint_velocity <= 0.0:
            raise ValueError("max_joint_velocity must be > 0")
        if self._max_joint_velocity_per_joint and len(self._max_joint_velocity_per_joint) != len(self._arm_joint_names):
            raise ValueError("max_joint_velocity_per_joint must be empty or the same length as arm_joint_names")

        # rad/tick step cap implied by the velocity limit(s) above, at the
        # nominal control_rate_hz - same conversion as i2rt_teleop's
        # leader_follower_node.cpp. control_rate_hz is nominal only (ticks
        # can run slower, e.g. under VLA inference latency, or be skipped
        # entirely on a failed tick) - the actual elapsed time between two
        # published commands is therefore always >= 1/control_rate_hz, so
        # this stays a true upper bound on achieved velocity, never an
        # underestimate.
        joint_velocity_limits = self._max_joint_velocity_per_joint or [self._max_joint_velocity] * len(self._arm_joint_names)
        self._max_step_per_joint = [v / self._control_rate_hz for v in joint_velocity_limits]

        self._enabled = False
        self._consecutive_ik_failures = 0
        self._ready_to_command = False  # true once the post-arm home ramp + hold has completed
        self._home_ramp_started = False
        self._home_ramp_and_hold_deadline = None
        self._latest_image = None
        self._latest_image_stamp = None
        self._last_image_fingerprint = None
        self._last_image_age_s = None
        self._latest_joint_state = None
        self._last_commanded_joint_positions = None
        self._last_commanded_gripper_position = None
        self._cv_bridge = CvBridge()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # _control_tick (below) blocks its calling thread for up to
        # request_timeout_s (HTTP) and ik_timeout_s+0.1 (busy-waiting on the
        # /compute_ik future). rclpy's DEFAULT callback group is mutually
        # exclusive, meaning nothing else in that same group - including the
        # very service-response callback that would mark the IK future done
        # - can run while _control_tick is executing. Using the node's
        # default group for everything would therefore self-deadlock every
        # IK call: _control_tick's busy-wait would spin until its timeout,
        # never seeing the response, every single tick. Fix: the timer gets
        # its OWN exclusive group (so ticks still can't overlap themselves),
        # and everything the timer callback either polls (the IK client) or
        # depends on staying fresh during a long tick (image/joint_state
        # subs, the enable service) gets a separate reentrant group so it
        # can be serviced concurrently by the MultiThreadedExecutor's other
        # worker threads.
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._io_group = ReentrantCallbackGroup()

        self._image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, 1, callback_group=self._io_group
        )
        self._joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_state_callback,
            rclpy.qos.qos_profile_sensor_data,
            callback_group=self._io_group,
        )

        self._joint_trajectory_pub = self.create_publisher(JointTrajectory, joint_trajectory_topic, 10)
        self._target_robot_state_pub = self.create_publisher(DisplayRobotState, target_robot_state_topic, 10)
        self._target_pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)
        # The EXACT bytes sent to /act (post crop+resize), so you can view what the model
        # actually sees (ros2 run rqt_image_view rqt_image_view, or an Image display in RViz) -
        # the single most direct way to tell "model is confused because the target isn't even in
        # frame" apart from "model has converged near a real target".
        self._debug_image_pub = self.create_publisher(Image, debug_image_topic, 1)

        self._ik_client = self.create_client(GetPositionIK, compute_ik_service, callback_group=self._io_group)
        self._gripper_action_client = ActionClient(self, GripperCommand, gripper_action_name, callback_group=self._io_group)

        self._enable_srv = self.create_service(SetBool, "~/enable", self._handle_enable, callback_group=self._io_group)

        self._timer = self.create_timer(1.0 / self._control_rate_hz, self._control_tick, callback_group=self._timer_group)

        self.get_logger().warn(
            "Starting DISARMED (enabled=False): will query the OpenVLA server, solve IK, and publish the RViz "
            "ghost/target pose, but will NOT command joint_trajectory_controller or gripper_controller until "
            "armed via: ros2 service call ~/enable std_srvs/srv/SetBool \"{data: true}\" "
            "(resolve '~' against this node's actual name/namespace)."
        )

    # ------------------------------------------------------------------ #
    # Subscriptions / service
    # ------------------------------------------------------------------ #

    def _image_callback(self, msg):
        try:
            self._latest_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self._latest_image_stamp = msg.header.stamp
        except CvBridgeError as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}", throttle_duration_sec=5.0)

    def _joint_state_callback(self, msg):
        self._latest_joint_state = msg

    def _handle_enable(self, request, response):
        self._enabled = bool(request.data)
        response.success = True
        response.message = (
            "VLA bridge ARMED - will ramp to home_joint_positions and hold before commanding "
            "joint_trajectory_controller/gripper_controller from VLA output."
            if self._enabled
            else "VLA bridge DISARMED - visualization only."
        )
        self.get_logger().warn(response.message)
        return response

    # ------------------------------------------------------------------ #
    # Home ramp (runs once, right after arming, before any VLA-driven command)
    # ------------------------------------------------------------------ #

    def _advance_homing(self):
        """Called from _control_tick while armed but not yet ready_to_command. Kicks off the
        ramp-to-home trajectory once (on the first call after arming), then just waits out the
        ramp+hold deadline on subsequent calls."""
        if not self._home_ramp_enabled:
            # Debugging workflow: the operator has manually driven the arm to a specific pose
            # (e.g. via command_pose_node) and wants VLA control to begin from exactly there,
            # not from home_joint_positions - skip the ramp/hold entirely, start immediately.
            self._ready_to_command = True
            self.get_logger().warn(
                "home_ramp_enabled=false: skipping the ramp to home_joint_positions - VLA-driven control begins "
                "immediately from the arm's current pose."
            )
            return
        if not self._home_ramp_started:
            self._start_home_ramp()
            return
        if time.monotonic() >= self._home_ramp_and_hold_deadline:
            self._ready_to_command = True
            self.get_logger().warn("Home ramp + hold complete - VLA-driven control begins now.")

    def _start_home_ramp(self):
        if self._latest_joint_state is None:
            self.get_logger().warn("Armed, but no joint_states yet; waiting before homing.", throttle_duration_sec=2.0)
            return
        current = dict(zip(self._latest_joint_state.name, self._latest_joint_state.position))
        try:
            start = [current[name] for name in self._arm_joint_names]
        except KeyError as exc:
            self.get_logger().warn(f"Armed, but joint_states is missing {exc}; waiting before homing.", throttle_duration_sec=2.0)
            return

        end = list(self._home_joint_positions)
        max_delta = max(abs(e - s) for s, e in zip(start, end))
        # Never let a large start/home gap imply a joint velocity above
        # home_max_joint_velocity, even if that means running longer than
        # home_ramp_duration_s - same reasoning as i2rt_teleop's
        # leader_follower_node.cpp start_ramp().
        min_duration_for_velocity = max_delta / self._home_max_joint_velocity
        duration = max(self._home_ramp_duration_s, min_duration_for_velocity)

        traj = JointTrajectory()
        traj.joint_names = list(self._arm_joint_names)
        n = self._home_ramp_waypoints
        for step in range(1, n + 1):
            frac = step / n
            point = JointTrajectoryPoint()
            point.positions = [s + frac * (e - s) for s, e in zip(start, end)]
            point.time_from_start = Duration(seconds=frac * duration).to_msg()
            traj.points.append(point)
        self._joint_trajectory_pub.publish(traj)
        # So the first real VLA-driven tick's velocity calc (see
        # _publish_joint_trajectory) is continuous from home, not a jump
        # computed against whatever start's stale value would otherwise be.
        self._last_commanded_joint_positions = end

        self._home_ramp_started = True
        self._home_ramp_and_hold_deadline = time.monotonic() + duration + self._home_hold_duration_s
        self.get_logger().warn(
            f"Ramping to home_joint_positions over {duration:.2f}s (max joint delta {max_delta:.3f} rad), then "
            f"holding {self._home_hold_duration_s:.1f}s before VLA-driven control begins. Keep the workspace clear."
        )

    # ------------------------------------------------------------------ #
    # Control loop
    # ------------------------------------------------------------------ #

    def _control_tick(self):
        if self._enabled and not self._ready_to_command:
            self._advance_homing()
            return

        if self._latest_image is None:
            self.get_logger().warn("No image yet on the image topic; skipping.", throttle_duration_sec=5.0)
            return
        if self._latest_joint_state is None:
            self.get_logger().warn("No joint_states yet; skipping.", throttle_duration_sec=5.0)
            return

        try:
            current_pos, current_quat = self._lookup_pose(self._base_link, self._ee_link)
            offset_pos, offset_quat = self._lookup_pose(self._ik_link_name, self._ee_link)
        except TransformException as exc:
            self.get_logger().warn(f"TF lookup failed ({exc}); skipping.", throttle_duration_sec=5.0)
            return

        action = self._query_vla_server()
        if action is None:
            return

        clamped_delta = pose_math.clamp_delta6(action[0:6], self._max_dxyz_m, self._max_drot_rad)
        self._log_action(action, clamped_delta)
        target_pos, target_quat = pose_math.integrate_delta(current_pos, current_quat, clamped_delta)
        link_target_pos, link_target_quat = pose_math.backsolve_parent_target(
            target_pos, target_quat, offset_pos, offset_quat
        )

        self._publish_target_pose(target_pos, target_quat)

        ik_solution = self._solve_ik(link_target_pos, link_target_quat)
        if ik_solution is None:
            return

        gripper_abs = float(np.clip(action[6], 0.0, 1.0))
        self._publish_ghost_state(ik_solution, gripper_abs)

        if not (self._enabled and self._ready_to_command):
            return

        self._publish_joint_trajectory(ik_solution)
        self._publish_gripper_command(gripper_abs)

    def _lookup_pose(self, target_frame, source_frame):
        """Returns (position_xyz, quat_xyzw) of `source_frame` expressed in `target_frame`."""
        transform = self._tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        t = transform.transform.translation
        q = transform.transform.rotation
        return np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w])

    def _preprocess_image(self, image):
        """Reproduces futur_vla/scripts/bag_to_npy.py's crop_and_resize exactly: center-crop the
        horizontal FOV to crop_width (full height kept), then resize to a square image_size."""
        h, w = image.shape[:2]
        if self._crop_width and self._crop_width < w:
            margin = (w - self._crop_width) // 2
            image = image[:, margin : margin + self._crop_width]
        resized = cv2.resize(image, (self._image_size, self._image_size), interpolation=cv2.INTER_AREA)
        return resized.astype(np.uint8)

    def _query_vla_server(self):
        image = self._preprocess_image(self._latest_image)
        # Fingerprint + age of the EXACT bytes sent to the server, so a run of repeated actions
        # can be told apart as "camera/scene genuinely static + deterministic decoding" (fingerprint
        # changes tick to tick, action doesn't) vs. "the image feed is actually frozen" (fingerprint
        # ALSO stays constant) - see the module-level note in _log_action.
        self._last_image_fingerprint = hashlib.md5(image.tobytes()).hexdigest()[:8]
        if self._latest_image_stamp is not None:
            image_time = rclpy.time.Time.from_msg(self._latest_image_stamp)
            self._last_image_age_s = (self.get_clock().now() - image_time).nanoseconds / 1e9
        try:
            debug_msg = self._cv_bridge.cv2_to_imgmsg(image, encoding="rgb8")
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            self._debug_image_pub.publish(debug_msg)
        except CvBridgeError as exc:
            self.get_logger().warn(f"Could not publish debug_image ({exc}).", throttle_duration_sec=10.0)
        instruction = self.get_parameter("instruction").value
        payload = {"image": image, "instruction": instruction, "unnorm_key": self._unnorm_key}
        try:
            response = requests.post(f"{self._server_url}/act", json=payload, timeout=self._request_timeout_s)
            response.raise_for_status()
            result = response.json()
            # deploy.py's /act returns `JSONResponse(action)` - the array ITSELF is the response body, not
            # {"action": array} (that line in its docstring just documents the response's conceptual schema).
            # On a server-side exception it instead returns the literal string "error" (see predict_action's
            # except clause) - check for that explicitly so it doesn't fall through as a confusing numpy error.
            if isinstance(result, str) and result == "error":
                raise RuntimeError("server-side exception in predict_action (see the server's own log for the traceback)")
            action = np.asarray(result, dtype=float)
        except Exception as exc:  # noqa: BLE001 - a flaky inference server must never take the control loop down
            self.get_logger().warn(f"OpenVLA request failed ({exc}); holding last commanded state.", throttle_duration_sec=2.0)
            return None
        if action.shape != (7,):
            self.get_logger().warn(
                f"OpenVLA server returned action shape {action.shape}, expected (7,); skipping.",
                throttle_duration_sec=2.0,
            )
            return None
        return action

    def _log_action(self, raw_action, clamped_delta):
        """Logs the raw VLA output alongside the post-clamp delta actually applied, every tick -
        so "the arm stopped moving" is diagnosable as either a genuinely-tiny model output (raw
        norms already small) or something our own clamping/integration is suppressing (raw large,
        clamped much smaller). Unthrottled: ticks are already seconds apart (inference-bound), so
        this isn't spammy."""
        raw_dxyz_mm = np.asarray(raw_action[0:3]) * 1000.0
        raw_drot_deg = np.degrees(raw_action[3:6])
        clamped_dxyz_mm = np.asarray(clamped_delta[0:3]) * 1000.0
        clamped_drot_deg = np.degrees(clamped_delta[3:6])
        was_clamped = not np.allclose(raw_action[0:6], clamped_delta, atol=1e-9)
        age_str = f"{self._last_image_age_s:.2f}s" if self._last_image_age_s is not None else "?"
        self.get_logger().info(
            f"img[fp={self._last_image_fingerprint} age={age_str}] "
            f"action: dxyz={np.round(raw_dxyz_mm, 2).tolist()}mm (|.|={np.linalg.norm(raw_dxyz_mm):.2f}mm) "
            f"drot={np.round(raw_drot_deg, 2).tolist()}deg (|.|={np.linalg.norm(raw_drot_deg):.2f}deg) "
            f"grip={float(raw_action[6]):.3f}"
            + (
                f" | CLAMPED to dxyz={np.round(clamped_dxyz_mm, 2).tolist()}mm drot={np.round(clamped_drot_deg, 2).tolist()}deg"
                if was_clamped
                else ""
            )
        )

    def _solve_ik(self, position, quat):
        request = GetPositionIK.Request()
        request.ik_request.group_name = self._ik_group_name
        request.ik_request.ik_link_name = self._ik_link_name
        request.ik_request.pose_stamped.header.frame_id = self._base_link
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose.position.x = float(position[0])
        request.ik_request.pose_stamped.pose.position.y = float(position[1])
        request.ik_request.pose_stamped.pose.position.z = float(position[2])
        request.ik_request.pose_stamped.pose.orientation.x = float(quat[0])
        request.ik_request.pose_stamped.pose.orientation.y = float(quat[1])
        request.ik_request.pose_stamped.pose.orientation.z = float(quat[2])
        request.ik_request.pose_stamped.pose.orientation.w = float(quat[3])
        request.ik_request.robot_state.joint_state = self._latest_joint_state
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=self._ik_timeout_s).to_msg()

        if not self._ik_client.service_is_ready():
            self.get_logger().warn(
                f"'{self._ik_client.srv_name}' service not available (is move_group running?); skipping.",
                throttle_duration_sec=5.0,
            )
            return None

        future = self._ik_client.call_async(request)
        deadline = time.monotonic() + self._ik_timeout_s + 0.1
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.001)
        if not future.done():
            self.get_logger().warn("/compute_ik call timed out; skipping.", throttle_duration_sec=2.0)
            return None

        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self._consecutive_ik_failures += 1
            seed = dict(zip(self._latest_joint_state.name, self._latest_joint_state.position))
            seed_arm = [round(seed.get(name, float("nan")), 4) for name in self._arm_joint_names]
            self.get_logger().warn(
                f"IK failed (error_code={response.error_code.val}, -31=NO_IK_SOLUTION) for target link_6 pose "
                f"pos={[round(float(v), 4) for v in position]} quat_xyzw={[round(float(v), 4) for v in quat]} "
                f"seeded from arm joints {list(self._arm_joint_names)}={seed_arm}; skipping ({self._consecutive_ik_failures} "
                f"consecutive failure(s)).",
                throttle_duration_sec=2.0,
            )
            # A near-zero-joint seed is common right after mock/real hardware
            # startup and is a textbook numeric-IK trap: joint2/joint3's
            # lower limit is exactly 0.0 (see i2rt.srdf), so sitting at or
            # near that limit is very often ALSO a kinematic singularity
            # (arm fully extended) where the local Jacobian is rank-deficient
            # - no amount of ik_timeout_s escapes that, since the solver
            # can't compute a sensible descent direction there at all, even
            # for a target millimeters away. Escalate periodically rather
            # than repeat the same per-tick warning forever.
            if self._consecutive_ik_failures % 5 == 0:
                self.get_logger().error(
                    f"{self._consecutive_ik_failures} consecutive IK failures. If the seed above is near all-zero "
                    "(especially joint2/joint3 near their 0.0 lower limit), this is likely a kinematic singularity, "
                    "not an out-of-workspace target - KDL's local solver can't escape one no matter how long it "
                    "runs. Move the arm to a well-conditioned pose first (e.g. the SRDF 'home' state: joint1=0, "
                    "joint2=1.5, joint3=1.5, joint4=0, joint5=0, joint6=0 - via RViz's MotionPlanning tab or a "
                    "one-off JointTrajectory) before arming this bridge.",
                    throttle_duration_sec=10.0,
                )
            return None

        self._consecutive_ik_failures = 0
        solution = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        try:
            return [solution[name] for name in self._arm_joint_names]
        except KeyError as exc:
            self.get_logger().warn(f"IK solution missing joint {exc}; skipping.", throttle_duration_sec=2.0)
            return None

    def _publish_target_pose(self, position, quat):
        msg = PoseStamped()
        msg.header.frame_id = self._base_link
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in position)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = (
            float(v) for v in quat
        )
        self._target_pose_pub.publish(msg)

    def _publish_ghost_state(self, ik_solution, gripper_abs):
        msg = DisplayRobotState()
        msg.state.joint_state.name = list(self._arm_joint_names) + [self._gripper_joint_name]
        gripper_target = self._gripper_closed_pos + gripper_abs * (self._gripper_open_pos - self._gripper_closed_pos)
        msg.state.joint_state.position = [float(v) for v in ik_solution] + [float(gripper_target)]
        ghost_color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.45)
        msg.highlight_links = [ObjectColor(id=link_name, color=ghost_color) for link_name in self._ghost_link_names]
        self._target_robot_state_pub.publish(msg)

    def _clamp_to_max_joint_velocity(self, ik_solution):
        """Slew-limits ik_solution relative to the last COMMANDED positions so no joint moves
        faster than max_joint_velocity (or its per-joint override) - same rate-limiting technique
        as i2rt_teleop's leader_follower_node.cpp mirror_tick(). This is the only joint-space
        velocity cap in the VLA-driven control path (see SAFETY MODEL in the module docstring for
        why the upstream Cartesian delta clamp isn't equivalent to this)."""
        if self._last_commanded_joint_positions is None:
            return list(ik_solution)
        clamped = []
        for target, last, max_step in zip(ik_solution, self._last_commanded_joint_positions, self._max_step_per_joint):
            delta = target - last
            if abs(delta) > max_step:
                target = last + copysign(max_step, delta)
            clamped.append(target)
        return clamped

    def _publish_joint_trajectory(self, ik_solution):
        clamped_solution = self._clamp_to_max_joint_velocity(ik_solution)

        traj = JointTrajectory()
        traj.joint_names = list(self._arm_joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in clamped_solution]
        if self._last_commanded_joint_positions is not None:
            # Non-zero velocity target so joint_trajectory_controller splines smoothly
            # instead of braking to zero-velocity every tick, then re-accelerating - same
            # reasoning as i2rt_teleop's leader_follower_node.cpp mirror_tick(). Computed from
            # the CLAMPED solution, so this always agrees with the max_joint_velocity cap above.
            point.velocities = [
                (v - last) * self._control_rate_hz
                for v, last in zip(clamped_solution, self._last_commanded_joint_positions)
            ]
        point.time_from_start = Duration(seconds=self._trajectory_time_from_start_s).to_msg()
        traj.points.append(point)
        self._joint_trajectory_pub.publish(traj)
        self._last_commanded_joint_positions = list(clamped_solution)

    def _publish_gripper_command(self, gripper_abs):
        target = self._gripper_closed_pos + gripper_abs * (self._gripper_open_pos - self._gripper_closed_pos)
        if (
            self._last_commanded_gripper_position is not None
            and abs(target - self._last_commanded_gripper_position) < self._gripper_position_threshold
        ):
            return  # Not enough change to justify preempting the controller's current goal.
        if not self._gripper_action_client.server_is_ready():
            self.get_logger().warn("Gripper action server not available; skipping gripper command.", throttle_duration_sec=5.0)
            return
        goal = GripperCommand.Goal()
        goal.command.position = target
        goal.command.max_effort = self._gripper_max_effort
        self._gripper_action_client.send_goal_async(goal)  # fire-and-forget: every new goal preempts the last.
        self._last_commanded_gripper_position = target


def main(args=None):
    rclpy.init(args=args)
    node = VlaBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
