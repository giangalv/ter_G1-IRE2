"""rclpy-free mapping from the workspace BODY-41 / hand vectors to URDF joints.

The VLA stack publishes ``/g1/joint_states`` as a flat 41-slot vector in the
BODY-41 order defined in ``CONVENTIONS.md`` (arms 0-13, waist 14-16, legs
17-28, reserved 29-40). RViz / robot_state_publisher instead want a
``sensor_msgs/JointState`` keyed by the URDF joint *names*. This module holds
the pure mapping logic; the ROS shell is `joint_state_relay.py`.

Default names follow the Unitree ``g1_description`` URDF (29-DOF variant).
Both lists are exposed as ROS parameters so any URDF flavor can be matched
without touching code: an EMPTY string in a name list drops that index.
"""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

# URDF joint names in BODY-41 index order (indices 0..28; 29-40 are reserved
# SDK padding and have no URDF counterpart).
DEFAULT_BODY_JOINT_NAMES: List[str] = [
    # 0-6 left arm
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    # 7-13 right arm
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    # 14-16 waist (yaw Z, roll X, pitch Y)
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    # 17-22 left leg
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    # 23-28 right leg
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]

# HAND-12 order (per CONVENTIONS.md): little, ring, middle, index, thumb_flex,
# thumb_rotation - left hand then right hand. Defaults are the six DRIVEN joints of the
# Inspire hands in Unitree's ``g1_29dof_rev_1_0_with_inspire_hand_FTP`` URDF (the other
# finger joints are ``mimic`` joints that robot_state_publisher derives). Set the
# parameters to your URDF's names for another model; an empty list leaves the fingers
# out of /joint_states.
DEFAULT_HAND_JOINT_NAMES_LEFT: List[str] = [
    "left_little_1_joint",
    "left_ring_1_joint",
    "left_middle_1_joint",
    "left_index_1_joint",
    "left_thumb_2_joint",
    "left_thumb_1_joint",
]
DEFAULT_HAND_JOINT_NAMES_RIGHT: List[str] = [
    n.replace("left_", "right_", 1) for n in DEFAULT_HAND_JOINT_NAMES_LEFT
]
# Joint angle (rad) at normalised position 1.0 (closed); 0.0 (open) is angle 0 in that URDF.
DEFAULT_HAND_JOINT_CLOSED_RAD: List[float] = [1.4381, 1.4381, 1.4381, 1.4381, 0.5864, 1.1641]


def hand_to_angles(
    positions: Sequence[float], closed_rad: Sequence[float]
) -> List[float]:
    """Normalised hand positions (0 open .. 1 closed) -> joint angles in radians.

    Values are clipped to [0, 1]; a missing position (short vector) is treated as open.
    """
    out: List[float] = []
    for i, limit in enumerate(closed_rad):
        p = float(positions[i]) if i < len(positions) else 0.0
        out.append(min(max(p, 0.0), 1.0) * float(limit))
    return out


def map_joint_vector(
    names: Sequence[str],
    positions: Sequence[float],
    velocities: Optional[Sequence[float]] = None,
) -> Tuple[List[str], List[float], List[float]]:
    """Map a flat joint vector onto URDF names, dropping empty-name slots.

    ``names`` is index-aligned with ``positions`` (extra positions beyond the
    name list — e.g. the reserved BODY-41 slots — are ignored). Velocities are
    optional; when absent, zeros are returned so consumers get aligned arrays.

    Returns (urdf_names, positions, velocities), all equal length.
    """
    pos = np.asarray(positions, dtype=np.float64)
    if velocities is not None and len(velocities) > 0:
        vel = np.asarray(velocities, dtype=np.float64)
    else:
        vel = np.zeros_like(pos)

    out_names: List[str] = []
    out_pos: List[float] = []
    out_vel: List[float] = []
    for i, name in enumerate(names):
        if not name or i >= pos.shape[0]:
            continue
        out_names.append(name)
        out_pos.append(float(pos[i]))
        out_vel.append(float(vel[i]) if i < vel.shape[0] else 0.0)
    return out_names, out_pos, out_vel


def merge_joint_states(
    *groups: Tuple[List[str], List[float], List[float]],
) -> Tuple[List[str], List[float], List[float]]:
    """Concatenate mapped groups (body + hands); later duplicates win."""
    index: dict = {}
    for names, positions, velocities in groups:
        for name, p, v in zip(names, positions, velocities):
            index[name] = (p, v)
    names_out = list(index.keys())
    return (
        names_out,
        [index[n][0] for n in names_out],
        [index[n][1] for n in names_out],
    )
