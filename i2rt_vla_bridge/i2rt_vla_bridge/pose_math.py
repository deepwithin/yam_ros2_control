"""Pure pose/frame math for the VLA bridge.

Deliberately free of any ROS message types or rclpy imports so it stays
trivially unit-testable (`python3 -c "from i2rt_vla_bridge import pose_math"`)
outside a full ROS environment.

All poses are `(position_xyz, quat_xyzw)` pairs of length-3 / length-4
array-likes. All 6-dim deltas are the OpenVLA action convention's first six
components: `[dx, dy, dz, droll, dpitch, dyaw]` (meters, then radians) - see
futur_vla/scripts/bag_to_npy.py, where the training-time delta is computed as

    r_rel = r2 * r1.inv()
    drot  = r_rel.as_euler('xyz')

i.e. `r_rel` is a WORLD-frame (left-multiplied / extrinsic) increment on top
of the pose at the previous step, not a body-frame one. `integrate_delta`
below reproduces that exact composition order so live inference integrates
deltas the same way they were generated during data collection.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def to_matrix(position_xyz, quat_xyzw):
    """(position_xyz, quat_xyzw) -> 4x4 homogeneous transform."""
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    matrix[:3, 3] = position_xyz
    return matrix


def from_matrix(matrix):
    """4x4 homogeneous transform -> (position_xyz, quat_xyzw)."""
    position_xyz = np.array(matrix[:3, 3], dtype=float)
    quat_xyzw = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return position_xyz, quat_xyzw


def integrate_delta(current_position_xyz, current_quat_xyzw, action6):
    """Apply an OpenVLA `[dx,dy,dz,droll,dpitch,dyaw]` delta onto a current
    pose, matching bag_to_npy.py's training-time convention
    (target_rot = r_rel * current_rot, i.e. left-multiplied)."""
    dxyz = np.asarray(action6[0:3], dtype=float)
    drot = np.asarray(action6[3:6], dtype=float)

    target_position_xyz = np.asarray(current_position_xyz, dtype=float) + dxyz

    r_rel = Rotation.from_euler("xyz", drot)
    r_cur = Rotation.from_quat(current_quat_xyzw)
    target_quat_xyzw = (r_rel * r_cur).as_quat()

    return target_position_xyz, target_quat_xyzw


def backsolve_parent_target(target_child_position_xyz, target_child_quat_xyzw, parent_to_child_position_xyz, parent_to_child_quat_xyzw):
    """Given a desired pose of a child frame C (e.g. gripper_tip) expressed
    in some reference frame R, and the CURRENT transform parent->C (e.g. the
    live link_6->gripper_tip TF, which already reflects the current gripper
    opening), solve for the pose of `parent` in R that would place C at the
    target:

        T_R_parent = T_R_C_target @ inverse(T_parent_C)

    Used to convert a VLA target expressed in the gripper_tip frame into an
    IK target for link_6 (the arm group's actual, unmoving-w.r.t.-gripper-
    state, IK tip), without needing to re-tip the SRDF/kinematics chain
    through the gripper's own (passive, IK-irrelevant) prismatic joint.
    """
    t_target_child = to_matrix(target_child_position_xyz, target_child_quat_xyzw)
    t_parent_child = to_matrix(parent_to_child_position_xyz, parent_to_child_quat_xyzw)
    t_target_parent = t_target_child @ np.linalg.inv(t_parent_child)
    return from_matrix(t_target_parent)


def clamp_delta6(action6, max_dxyz, max_drot_rad):
    """Clamp the translation/rotation magnitude of a 6-dim delta to safety
    limits, preserving direction. A safety net against a bad/out-of-
    distribution model output, independent of whatever the model itself
    produces - mirrors the velocity-clamping in i2rt_teleop's
    leader_follower_node.cpp."""
    dxyz = np.asarray(action6[0:3], dtype=float)
    drot = np.asarray(action6[3:6], dtype=float)

    dxyz_norm = np.linalg.norm(dxyz)
    if dxyz_norm > max_dxyz and dxyz_norm > 0:
        dxyz = dxyz * (max_dxyz / dxyz_norm)

    drot_norm = np.linalg.norm(drot)
    if drot_norm > max_drot_rad and drot_norm > 0:
        drot = drot * (max_drot_rad / drot_norm)

    return np.concatenate([dxyz, drot])
