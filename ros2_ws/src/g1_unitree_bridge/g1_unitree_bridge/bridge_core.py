"""rclpy-free mapping from Unitree's G1 motor order to the workspace BODY-41 vector.

``unitree_hg/LowState.motor_state`` is indexed by Unitree's G1 joint numbering
(legs 0-11, waist 12-14, left arm 15-21, right arm 22-28; slots 29-34 are
unused on the 29-DoF robot). The workspace (CONVENTIONS.md, "Joint index maps")
orders the same joints as arms, waist, legs, followed by 12 reserved slots.
"""
from __future__ import annotations

from typing import List, Sequence, Tuple

BODY_DIM = 41
N_JOINTS = 29

# BODY-41 index -> index in LowState.motor_state.
BODY41_TO_UNITREE: List[int] = (
    list(range(15, 22))  # 0-6   left arm  (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw)
    + list(range(22, 29))  # 7-13  right arm
    + [12, 13, 14]  # 14-16 waist yaw, roll, pitch
    + list(range(0, 6))  # 17-22 left leg  (hip pitch/roll/yaw, knee, ankle pitch/roll)
    + list(range(6, 12))  # 23-28 right leg
)

assert len(BODY41_TO_UNITREE) == N_JOINTS and len(set(BODY41_TO_UNITREE)) == N_JOINTS

_LEG = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
_ARM = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")

# Same names as g1_viz.relay_core.DEFAULT_BODY_JOINT_NAMES (Unitree URDF), then the reserved slots.
BODY41_NAMES: List[str] = (
    [f"left_{j}_joint" for j in _ARM]
    + [f"right_{j}_joint" for j in _ARM]
    + ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
    + [f"left_{j}_joint" for j in _LEG]
    + [f"right_{j}_joint" for j in _LEG]
    + [f"reserved_{i}" for i in range(N_JOINTS, BODY_DIM)]
)
assert len(BODY41_NAMES) == BODY_DIM


def to_body41(values: Sequence[float]) -> List[float]:
    """Reorder a per-motor array (Unitree order) into a 41-slot BODY-41 vector.

    Slots 29-40 are the reserved SDK slots and stay 0.0. Raises ValueError when
    the input is too short to contain the 29 actuated joints.
    """
    if len(values) < max(BODY41_TO_UNITREE) + 1:
        raise ValueError(f"need at least {max(BODY41_TO_UNITREE) + 1} motors, got {len(values)}")
    out = [0.0] * BODY_DIM
    for body_idx, motor_idx in enumerate(BODY41_TO_UNITREE):
        out[body_idx] = float(values[motor_idx])
    return out


def quat_wxyz_to_xyzw(q: Sequence[float]) -> Tuple[float, float, float, float]:
    """Unitree IMU quaternions are (w, x, y, z); ROS wants (x, y, z, w)."""
    return float(q[1]), float(q[2]), float(q[3]), float(q[0])
