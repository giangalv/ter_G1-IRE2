"""Tests for the rclpy-free Unitree -> BODY-41 joint reordering."""
from __future__ import annotations

import pytest

from g1_unitree_bridge.bridge_core import (
    BODY41_NAMES,
    BODY41_TO_UNITREE,
    BODY_DIM,
    N_JOINTS,
    quat_wxyz_to_xyzw,
    to_body41,
)


def test_mapping_is_a_permutation_of_the_29_actuated_motors() -> None:
    assert sorted(BODY41_TO_UNITREE) == list(range(N_JOINTS))


def test_names_match_the_slots() -> None:
    assert len(BODY41_NAMES) == BODY_DIM
    assert BODY41_NAMES[3] == "left_elbow_joint"
    assert BODY41_NAMES[10] == "right_elbow_joint"
    assert BODY41_NAMES[14] == "waist_yaw_joint"
    assert BODY41_NAMES[20] == "left_knee_joint"
    assert BODY41_NAMES[26] == "right_knee_joint"
    assert BODY41_NAMES[29] == "reserved_29"


def test_reordering_uses_unitree_indices() -> None:
    motors = [float(i) for i in range(35)]  # value == Unitree motor index
    body = to_body41(motors)
    assert len(body) == BODY_DIM
    assert body[0] == 15.0  # left shoulder pitch is Unitree motor 15
    assert body[3] == 18.0  # left elbow
    assert body[7] == 22.0  # right shoulder pitch
    assert body[14] == 12.0  # waist yaw
    assert body[17] == 0.0  # left hip pitch
    assert body[20] == 3.0  # left knee
    assert body[23] == 6.0  # right hip pitch
    assert body[28] == 11.0  # right ankle roll


def test_reserved_slots_are_zero() -> None:
    body = to_body41([1.0] * 35)
    assert body[N_JOINTS:] == [0.0] * (BODY_DIM - N_JOINTS)


def test_short_input_is_rejected() -> None:
    with pytest.raises(ValueError):
        to_body41([0.0] * 20)


def test_quaternion_order() -> None:
    assert quat_wxyz_to_xyzw([0.9, 0.1, 0.2, 0.3]) == (0.1, 0.2, 0.3, 0.9)


def test_gst_pipeline_names_the_group_and_nic() -> None:
    from g1_unitree_bridge.head_video_node import gst_pipeline

    p = gst_pipeline("230.1.1.1", 1720, "enp59s0")
    assert "address=230.1.1.1 port=1720 multicast-iface=enp59s0" in p
    assert "rtph264depay" in p and p.endswith("sync=false")
    assert "multicast-iface" not in gst_pipeline("230.1.1.1", 1720, "")
