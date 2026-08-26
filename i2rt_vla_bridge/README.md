# i2rt_vla_bridge

Live OpenVLA -> IK -> YAM arm bridge, plus RViz showing the next IK-solved
step as a ghost alongside the live arm. This file is the design plan this
package was built from, kept here for later reference, followed by concrete
run instructions.

## Quickstart

Prerequisites (not started by this package):

1. Arm bringup - `ros2 launch i2rt_moveit_config demo.launch.py` (defaults to
   mock hardware; safe to try first). This provides `robot_state_publisher`,
   `ros2_control`, `joint_trajectory_controller`/`gripper_controller`, and
   `move_group` (for `/compute_ik`).
2. A camera publishing `sensor_msgs/Image` on `/camera/camera/color/image_raw`
   (e.g. a `realsense2_camera` launch).
3. The OpenVLA inference server, pointed at a `futur_vla` checkpoint:
   ```
   python vla-scripts/deploy.py --openvla_path ~/WORKSHOP/futur_vla/runs/openvla-7b+yam_packaging+.../4500_chkpt
   ```
4. Python deps for this package: `pip install -r requirements.txt`.

Then:
```
ros2 launch i2rt_vla_bridge vla_bridge.launch.py instruction:="pick up the red block"
```
This starts the bridge (DISARMED - see below) and RViz with the ghost
display. Confirm the ghost tracks sane targets before arming:
```
ros2 service call /vla_bridge_node/enable std_srvs/srv/SetBool "{data: true}"
```
(disarm the same way with `{data: false}`). Arming does not immediately hand
control to VLA output - the arm first ramps to `home_joint_positions` (a
node parameter, defaults to a known-good debug pose from
`i2rt_teleop/src/command_pose_node.cpp`) and holds there
(`home_ramp_duration_s` / `home_hold_duration_s`, both launch args), so VLA
control never starts from an arbitrary - and often near-singular - pose. Set
`home_ramp_enabled:=false` to skip this and begin VLA-driven control
immediately from wherever the arm currently is - useful when debugging model
output from a specific, manually-driven pose.

**Current hardware note:** while the real arm is running under the existing
dual-arm leader/follower teleop (`i2rt_teleop/launch/leader_follower.launch.py`),
do not point this bridge at it - that launch doesn't run `move_group`, and
this repo's TF/robot_state_publisher setup isn't namespace-safe for two arms
running at once (see Limitations below). Use `i2rt_moveit_config/demo.launch.py`
(single, unnamespaced arm) for now.

## Design plan

### Context

The user has a trained OpenVLA checkpoint (`~/WORKSHOP/futur_vla`, fine-tuned
from `~/WORKSHOP/openvla`) that outputs 7-dim delta end-effector actions, and
a ROS2 (Jazzy) control stack for the YAM arm in `yam_ros2_control` with
MoveIt/KDL IK already configured but no Cartesian/IK-calling code and no VLA
integration. The goal is a live inference loop: camera image -> OpenVLA
server -> delta EE pose -> IK -> arm execution, with RViz showing both the
live arm and the next IK-solved target as a ghost.

Key facts established during exploration:

- **OpenVLA server** (`openvla/vla-scripts/deploy.py`) already exists and is
  reusable as-is: FastAPI, `POST /act` with
  `{"image": ndarray, "instruction": str, "unnorm_key": str}` -> the 7-dim
  action **array itself** as the response body (`JSONResponse(action)` -
  *not* wrapped as `{"action": ndarray}`, despite that being how the
  docstring describes the schema conceptually), decoded via `json_numpy`
  wire encoding on both ends. On a server-side exception the body is instead
  the literal string `"error"`. Point `openvla_path` at a
  `futur_vla/runs/.../*_chkpt` directory (auto-loads its
  `dataset_statistics.json`); pass `unnorm_key="yam_packaging"`.
- **Action format** (from `futur_vla/scripts/bag_to_npy.py`, matches
  `openvla/experiments/robot/robot_utils.py` `ACTION_DIM=7`):
  `[dx, dy, dz, droll, dpitch, dyaw, gripper_abs]`. Translation deltas in
  meters, rotation deltas as **Euler xyz** in radians, computed as
  `r_rel = r2 * r1.inv()` between consecutive resampled steps -> so
  integration is `target_rot = r_rel * current_rot` (left-multiply /
  world-frame composition), `target_pos = current_pos + [dx,dy,dz]`. Gripper
  is **absolute** [0,1] (0=closed,1=open); `mask[6]=false` in
  `dataset_statistics.json` confirms it's not q01/q99-rescaled.
- **Frame**: pose is `base_link -> gripper_tip`
  (`bag_to_npy.py --ee_link gripper_tip --base_link base_link`), which
  already matches this repo's `gripper_tip` link
  (`i2rt_description/urdf/gripper_linear_4310.xacro:75-81`) - no
  re-derivation of the target frame needed, just consistent use of it.
- **Control rate**: training used 5 Hz resampling, no action chunking - the
  bridge runs its loop at ~5 Hz.
- **Gripper calibration**: raw `gripper_joint` range is `closed=0.0`,
  `open=-0.0475` (same constants `bag_to_npy.py` used, and the URDF's joint
  limits) - used to map `gripper_abs` back to a raw joint command.
- **ROS2 side**: `i2rt_moveit_config` already configures
  `kdl_kinematics_plugin/KDLKinematicsPlugin` for group `arm` (chain
  `base_link`->`link_6`), and `move_group` (launched by `demo.launch.py`)
  already advertises `/compute_ik` (`moveit_msgs/srv/GetPositionIK`). No
  existing IK-calling code anywhere; all current motion (`i2rt_teleop`) is
  joint-space only (`JointTrajectory` + `GripperCommand` action).
- **Why not retip the SRDF to `gripper_tip`**: `gripper_tip` sits past the
  *prismatic* `gripper_joint` (child of `tip_right`), so if it were the IK
  chain's tip, IK would solve 7 DOF (6 arm + gripper) instead of 6, coupling
  gripper position to arm IK incorrectly. Instead: keep IK targeting `link_6`
  (already configured), and use the *live* TF transform `link_6 ->
  gripper_tip` (which reflects the current gripper opening) to convert the
  VLA's `gripper_tip`-frame target into a `link_6` target before calling
  `/compute_ik`. This requires zero SRDF/kinematics.yaml changes.
- **Camera topic**: `/camera/camera/color/image_raw` (RealSense,
  `sensor_msgs/Image`, matches `bag_to_npy.py` default and
  `record_episode.sh`). Not launched by this repo - assumed to come from a
  `realsense2_camera` launch run separately, same as during data collection.
- Bridge node is **Python (rclpy)** (HTTP/JSON/quaternion math is native
  there vs. C++ needing libcurl + manual json_numpy codec, a deliberate
  exception to the rest of this repo's C++/ament_cmake convention), and
  **single-arm**, namespace-configurable (default no namespace, matching
  `demo.launch.py`'s un-namespaced single-arm setup; settable to
  `follower`/`leader` if a namespaced move_group is separately arranged).

### Package layout

```
i2rt_vla_bridge/
  package.xml, setup.py, setup.cfg
  i2rt_vla_bridge/
    vla_bridge_node.py       # main node - see its module docstring for the full pipeline + safety model
    pose_math.py             # pure-function helpers: pose<->matrix, delta integration, offset back-solve
  launch/
    vla_bridge.launch.py
  config/
    vla_inference.rviz       # moveit.rviz + ghost RobotState display + target pose axes
  requirements.txt           # requests, json-numpy, scipy (pip deps rosdep won't cover)
```

### Control loop (see `vla_bridge_node.py`'s module docstring for the authoritative version)

Per tick (`control_rate_hz`, default 5.0 Hz):

1. Look up `base_link->gripper_tip` (current EE pose) and `link_6->
   gripper_tip` (current gripper-dependent offset) via TF.
2. Preprocess the latest camera frame the same way training data was built
   (`bag_to_npy.py`'s `crop_and_resize`: center-crop to `crop_width` then
   resize to `image_size`x`image_size`).
3. `POST {server_url}/act` with `{image, instruction, unnorm_key}`
   (`json_numpy.patch()`, exactly as `deploy.py`'s docstring documents). On
   failure: log + skip this tick, never crash the loop.
4. Clamp the 6-dim delta's magnitude (`max_dxyz_m` / `max_drot_deg`) as a
   safety net, then integrate onto the current `gripper_tip` pose
   (`pose_math.integrate_delta`).
5. Back-solve the equivalent `link_6` target
   (`pose_math.backsolve_parent_target`).
6. Call `/compute_ik` (group `arm`, tip `link_6`, seeded from the live
   joint state).
7. Publish a `moveit_msgs/DisplayRobotState` ghost of the solved target on
   `vla_bridge/target_robot_state` - **always**, even while disarmed, so the
   ghost is visible for preview before arming.
8. If armed: stream the solution to `joint_trajectory_controller` and the
   gripper's absolute command to `gripper_controller`'s `GripperCommand`
   action.

### Safety model

- Starts **disarmed** (`enabled=False`). Arm via
  `ros2 service call ~/enable std_srvs/srv/SetBool "{data: true}"` - mirrors
  the `alignment_confirmed` gate in `i2rt_teleop/src/leader_follower_node.cpp`.
- Arming does not immediately hand control to VLA output: the real arm first
  ramps smoothly to `home_joint_positions` and holds
  (`home_ramp_duration_s`/`home_hold_duration_s`, velocity-clamped by
  `home_max_joint_velocity` the same way `leader_follower_node.cpp`'s
  startup ramp is) - only once that completes does VLA-driven commanding
  begin. Ghost/target-pose preview keeps working before and during this,
  since it never depends on `ready_to_command`.
- Per-tick delta clamps (`max_dxyz_m`, `max_drot_deg`) independent of
  whatever the model outputs.
- A flaky server call, TF failure, or IK failure all mean "skip this tick,
  hold the last commanded state" - never a crash, never a partial/malformed
  command.
- IK is always seeded from the arm's *live* current joint state, not an
  internally-integrated target, so a stall or skipped tick can't drift the
  seed from reality.

### Debugging model behavior (not IK/plumbing)

Once IK/plumbing checks out and the arm still doesn't do anything useful,
the bridge gives you two live diagnostics instead of guessing:

- Every tick's `INFO` log (`img[fp=... age=...] action: ...`) shows the raw
  VLA output, an md5 fingerprint of the exact image bytes sent to `/act`,
  and that image's age. A changing fingerprint with a stuck/near-zero action
  means the model itself is choosing not to move for a live, current scene -
  not a frozen camera feed (a constant fingerprint would mean that instead).
- `/vla_bridge/debug_image` (`sensor_msgs/Image`, also in the shipped RViz
  config as "VLA Debug Image (what the model sees)") publishes the *exact*
  224x224 post-crop frame sent to the server every tick - the fastest way to
  check whether the target object is even in frame given the current
  `crop_width`/arm pose, rather than inferring it from action numbers alone.

### Limitations / known gaps

- **Dual-arm namespacing is not solved here.** TF frames
  (`base_link`, `link_6`, `gripper_tip`, ...) are unprefixed and global
  regardless of which namespace's `robot_state_publisher` published them -
  this is a pre-existing repo limitation (see `i2rt_teleop`'s dual-arm
  namespacing notes), not something this package fixes. Running two arms'
  `robot_state_publisher`s at once makes TF lookups ambiguous. Use this
  bridge against a single, unnamespaced arm (`i2rt_moveit_config/
  demo.launch.py`) until/unless `tf_prefix` support is added upstream.
- `gripper_tip`'s offset (`gripper_linear_4310.xacro:75-81`) was derived by
  mesh-centroid heuristic, not measured - the commit that added it flags this
  as unverified against real hardware. Sanity-check it (view the frame's axes
  in RViz, or measure the physical fingertip-to-`tip_right`-origin distance)
  before trusting this bridge for precision grasping.
- The bridge's `/compute_ik` call blocks its control-loop thread for up to
  `ik_timeout_s + 0.1s` per tick (a `MultiThreadedExecutor` with 4 worker
  threads keeps other callbacks - image/joint_state subscriptions, the enable
  service - responsive in the meantime).
- **Start the arm from a well-conditioned pose, not all-zero joints.**
  `joint2`/`joint3`'s lower limit is exactly `0.0` (`i2rt.srdf`), so mock/real
  hardware's typical all-zero startup state is very often ALSO a kinematic
  singularity (arm fully extended) - KDL's local numeric solver can fail
  there (`NO_IK_SOLUTION`, error_code -31) even for a target millimeters
  away from where the arm already sits, no matter how large `ik_timeout_s`
  is (verified: FK'd the exact failing seed directly and the requested
  target was 3.5mm/1.45° away - not a workspace/reachability problem, a
  solver-at-a-singularity problem). Before arming, move the arm to the SRDF
  `home` state (`joint1=0, joint2=1.5, joint3=1.5, joint4=0, joint5=0,
  joint6=0`) via RViz's MotionPlanning tab or a one-off `JointTrajectory`.
  The node itself escalates to an `ERROR`-level hint with this same guidance
  every 5th consecutive IK failure.

## Verification

1. `colcon build --packages-select i2rt_vla_bridge` (plus existing packages)
   succeeds.
2. Bring up mock hardware first:
   `ros2 launch i2rt_moveit_config demo.launch.py` (default
   `use_mock_hardware:=true`) so `/compute_ik`, controllers, and TF are live
   without real motors.
3. Start a stub/real OpenVLA server and a camera publisher (real RealSense,
   or a static test image publisher on `/camera/camera/color/image_raw` for
   a dry run).
4. `ros2 launch i2rt_vla_bridge vla_bridge.launch.py instruction:="pick up
   the red block"` - confirm RViz opens showing the live (solid) arm and,
   once armed, the orange ghost tracking IK targets.
5. Arm it and confirm `/compute_ik` calls succeed
   (`ros2 topic hz /vla_bridge/target_robot_state` near 5 Hz) and the mock
   arm's `/joint_states` follows the ghost.
6. Only after mock validation, and only once the real arm is free (not
   mid-session under the dual-arm teleop launch), re-launch
   `demo.launch.py use_mock_hardware:=false` against real hardware, arm-gate
   still defaulting to disabled, and enable deliberately.
