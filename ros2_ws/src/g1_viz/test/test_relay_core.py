"""Tests for the rclpy-free BODY-41 -> URDF joint mapping."""
from __future__ import annotations

import numpy as np
import pytest

from g1_viz.relay_core import (
    DEFAULT_BODY_JOINT_NAMES,
    map_joint_vector,
    merge_joint_states,
)


def test_default_body_names_cover_29_actuated_joints() -> None:
    assert len(DEFAULT_BODY_JOINT_NAMES) == 29
    assert len(set(DEFAULT_BODY_JOINT_NAMES)) == 29  # no duplicates


def test_reserved_slots_are_dropped() -> None:
    positions = np.arange(41, dtype=float)
    names, pos, vel = map_joint_vector(DEFAULT_BODY_JOINT_NAMES, positions)
    assert len(names) == len(pos) == len(vel) == 29
    # index alignment preserved: slot 3 is the left elbow
    assert names[3] == "left_elbow_joint"
    assert pos[3] == 3.0


def test_missing_velocities_yield_zeros() -> None:
    _, _, vel = map_joint_vector(DEFAULT_BODY_JOINT_NAMES, np.ones(41), None)
    assert vel == [0.0] * 29
    _, _, vel = map_joint_vector(DEFAULT_BODY_JOINT_NAMES, np.ones(41), [])
    assert vel == [0.0] * 29


def test_velocities_pass_through() -> None:
    _, _, vel = map_joint_vector(
        DEFAULT_BODY_JOINT_NAMES, np.zeros(41), np.full(41, 0.5)
    )
    assert vel == [0.5] * 29


def test_empty_names_skip_indices() -> None:
    names = ["a", "", "c"]
    out_names, out_pos, _ = map_joint_vector(names, [1.0, 2.0, 3.0])
    assert out_names == ["a", "c"]
    assert out_pos == [1.0, 3.0]


def test_short_position_vector_is_truncated_not_fatal() -> None:
    out_names, out_pos, _ = map_joint_vector(["a", "b", "c"], [1.0, 2.0])
    assert out_names == ["a", "b"]
    assert out_pos == [1.0, 2.0]


def test_merge_concatenates_and_deduplicates() -> None:
    body = (["a", "b"], [1.0, 2.0], [0.0, 0.0])
    hand = (["h1", "h2"], [3.0, 4.0], [0.1, 0.2])
    dup = (["b"], [9.0], [9.0])  # later group wins
    names, pos, vel = merge_joint_states(body, hand, dup)
    assert names == ["a", "b", "h1", "h2"]
    assert pos == [1.0, 9.0, 3.0, 4.0]
    assert vel == [0.0, 9.0, 0.1, 0.2]


def test_merge_empty_groups() -> None:
    names, pos, vel = merge_joint_states()
    assert names == [] and pos == [] and vel == []


@pytest.mark.parametrize("index,name", [(0, "left_shoulder_pitch_joint"),
                                        (14, "waist_yaw_joint"),
                                        (17, "left_hip_pitch_joint"),
                                        (28, "right_ankle_roll_joint")])
def test_body41_landmark_indices(index: int, name: str) -> None:
    assert DEFAULT_BODY_JOINT_NAMES[index] == name


def test_hand_defaults_are_the_six_driven_inspire_joints() -> None:
    from g1_viz.relay_core import (
        DEFAULT_HAND_JOINT_CLOSED_RAD,
        DEFAULT_HAND_JOINT_NAMES_LEFT,
        DEFAULT_HAND_JOINT_NAMES_RIGHT,
    )

    assert len(DEFAULT_HAND_JOINT_NAMES_LEFT) == len(DEFAULT_HAND_JOINT_NAMES_RIGHT) == 6
    assert len(DEFAULT_HAND_JOINT_CLOSED_RAD) == 6
    assert DEFAULT_HAND_JOINT_NAMES_RIGHT[3] == "right_index_1_joint"
    assert DEFAULT_HAND_JOINT_NAMES_LEFT[4] == "left_thumb_2_joint"  # thumb flexion


def test_hand_to_angles_scales_and_clips() -> None:
    from g1_viz.relay_core import hand_to_angles

    closed = [1.0, 2.0, 4.0]
    assert hand_to_angles([0.0, 0.5, 1.0], closed) == [0.0, 1.0, 4.0]
    assert hand_to_angles([-0.2, 1.5, 0.25], closed) == [0.0, 2.0, 1.0]  # clipped to [0, 1]
    assert hand_to_angles([], closed) == [0.0, 0.0, 0.0]  # no message yet = open
