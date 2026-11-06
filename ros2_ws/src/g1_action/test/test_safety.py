"""Tests for the five-stage safety pipeline (numpy only, no rclpy)."""

import numpy as np
import pytest

from g1_action.executor_core import ACTION_DIM, BODY_DIM
from g1_action.safety import (
    REASON_DUAL_ENCODER_FAULT,
    REASON_ESTOP_LATCHED,
    REASON_OK,
    REASON_SELF_COLLISION_BLOCK,
    REASON_TORQUE_CLAMPED,
    REASON_VELOCITY_CLAMPED,
    REASON_WORKSPACE_BLOCK,
    SafetyPipeline,
    segment_segment_distance,
)


def _pipeline(**overrides) -> SafetyPipeline:
    return SafetyPipeline(**overrides)


def _neutral():
    """Target/prev/measured all at the neutral pose (arms hanging)."""
    target = np.zeros(ACTION_DIM)
    prev = np.zeros(ACTION_DIM)
    body = np.zeros(BODY_DIM)
    return target, prev, body


class TestDualEncoderFaultInjection:
    def test_fault_injection_estops_and_latches(self):
        rng = np.random.default_rng(7)
        pipeline = _pipeline(dual_encoder_fault_threshold=0.05)
        target, prev, primary = _neutral()

        # Clean frame executes.
        clean = pipeline.check(target, prev, primary, primary.copy())
        assert clean.ok
        assert not clean.estop

        # Inject noise above the threshold on one joint of the secondary.
        secondary = primary.copy()
        joint = int(rng.integers(0, 29))
        secondary[joint] += 0.05 + float(rng.uniform(0.01, 0.2))
        faulted = pipeline.check(target, prev, primary, secondary)
        assert not faulted.ok
        assert faulted.estop
        assert REASON_DUAL_ENCODER_FAULT in faulted.reason
        assert np.allclose(faulted.action, prev)  # hold

        # Latched across subsequent clean frames.
        for _ in range(5):
            again = pipeline.check(target, prev, primary, primary.copy())
            assert again.estop
            assert not again.ok
            assert REASON_ESTOP_LATCHED in again.reason
        assert pipeline.estop_latched

        # Explicit reset clears the latch.
        pipeline.reset()
        assert not pipeline.estop_latched
        recovered = pipeline.check(target, prev, primary, primary.copy())
        assert recovered.ok

    def test_below_threshold_noise_does_not_trip(self):
        pipeline = _pipeline(dual_encoder_fault_threshold=0.05)
        target, prev, primary = _neutral()
        secondary = primary.copy()
        secondary[:29] += 0.04  # everywhere below threshold
        decision = pipeline.check(target, prev, primary, secondary)
        assert decision.ok
        assert not pipeline.estop_latched

    def test_encoder_delta_helper(self):
        primary = np.zeros(BODY_DIM)
        secondary = np.zeros(BODY_DIM)
        secondary[3] = -0.2
        delta = SafetyPipeline.encoder_delta(primary, secondary)
        assert delta.shape == (BODY_DIM,)
        assert delta[3] == pytest.approx(0.2)
        assert np.all(delta[np.arange(BODY_DIM) != 3] == 0.0)


class TestVelocityClamp:
    def test_delta_clamped_to_rate_limit(self):
        pipeline = _pipeline(rate_hz=50.0, max_joint_velocity_rad_s=2.5)
        target, prev, primary = _neutral()
        target[2] = 1.0  # huge jump on left shoulder_yaw
        decision = pipeline.check(target, prev, primary, primary.copy())
        assert decision.ok
        assert REASON_VELOCITY_CLAMPED in decision.reason
        assert decision.action[2] == pytest.approx(2.5 / 50.0)

    def test_negative_delta_clamped_symmetrically(self):
        pipeline = _pipeline(rate_hz=50.0, max_joint_velocity_rad_s=2.5)
        target, prev, primary = _neutral()
        target[10] = -1.0
        decision = pipeline.check(target, prev, primary, primary.copy())
        assert decision.action[10] == pytest.approx(-0.05)

    def test_small_delta_untouched(self):
        pipeline = _pipeline()
        target, prev, primary = _neutral()
        target[4] = 0.01
        primary[4] = 0.01  # keep torque stage quiet
        decision = pipeline.check(target, prev, primary, primary.copy())
        assert decision.ok
        assert decision.reason == REASON_OK
        assert decision.action[4] == pytest.approx(0.01)


class TestTorqueClamp:
    def test_torque_demand_clamped_to_120nm(self):
        pipeline = _pipeline(
            max_joint_torque_nm=120.0, stiffness_nm_per_rad=80.0
        )
        target, prev, primary = _neutral()
        # Elbow commanded 2 rad away from measured; prev equals target so the
        # velocity stage does not interfere.
        target[3] = 2.0
        prev[3] = 2.0
        decision = pipeline.check(target, prev, primary, primary.copy())
        assert decision.ok
        assert REASON_TORQUE_CLAMPED in decision.reason
        assert decision.action[3] == pytest.approx(120.0 / 80.0)
        torque_estimate = 80.0 * abs(decision.action[3] - primary[3])
        assert torque_estimate <= 120.0 + 1e-9


class TestWorkspaceBlock:
    def test_forward_reach_outside_bounds_blocks(self):
        pipeline = _pipeline(
            workspace_bounds=(-0.2, 0.3, -0.6, 0.6, 0.0, 1.6)
        )
        target, prev, primary = _neutral()
        # Arm straight forward: wrist x ~ 0.54 m > x_max = 0.3.
        target[0] = np.pi / 2.0
        prev[:] = target
        primary[:] = target[:BODY_DIM]
        decision = pipeline.check(target, prev, primary, primary.copy())
        assert not decision.ok
        assert decision.blocked
        assert REASON_WORKSPACE_BLOCK in decision.reason
        assert np.allclose(decision.action, prev)  # hold

    def test_neutral_pose_inside_default_bounds(self):
        pipeline = _pipeline()
        target, prev, primary = _neutral()
        ok, wrists = pipeline.check_workspace(target[:BODY_DIM])
        assert ok
        assert wrists.shape == (2, 3)
        # Hanging arms: wrist z = shoulder_z - upper - fore.
        assert wrists[0, 2] == pytest.approx(1.1 - 0.28 - 0.26)


class TestSegmentSegmentDistance:
    def test_parallel_segments(self):
        d = segment_segment_distance(
            [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]
        )
        assert d == pytest.approx(1.0)

    def test_crossing_skew_segments(self):
        d = segment_segment_distance(
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.5, -1.0, 0.3],
            [0.5, 1.0, 0.3],
        )
        assert d == pytest.approx(0.3)

    def test_degenerate_point_point(self):
        d = segment_segment_distance(
            [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [1.0, 1.0, 1.0]
        )
        assert d == pytest.approx(np.sqrt(3.0))

    def test_degenerate_point_segment(self):
        d = segment_segment_distance(
            [0.0, 2.0, 0.0], [0.0, 2.0, 0.0], [-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]
        )
        assert d == pytest.approx(2.0)

    def test_collinear_overlapping_segments(self):
        d = segment_segment_distance(
            [0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 0.0, 0.0], [3.0, 0.0, 0.0]
        )
        assert d == pytest.approx(0.0)

    def test_endpoint_clamping(self):
        # Closest points forced onto segment endpoints.
        d = segment_segment_distance(
            [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 1.0, 0.0], [3.0, 1.0, 0.0]
        )
        assert d == pytest.approx(np.sqrt(2.0))


class TestSelfCollisionBlock:
    def test_arm_swung_across_torso_blocks(self):
        pipeline = _pipeline()
        target, prev, primary = _neutral()
        # Right shoulder roll swings the hanging arm across the torso axis.
        target[8] = 0.6
        prev[:] = target
        primary[:] = target[:BODY_DIM]
        decision = pipeline.check(target, prev, primary, primary.copy())
        assert not decision.ok
        assert decision.blocked
        assert REASON_SELF_COLLISION_BLOCK in decision.reason
        assert np.allclose(decision.action, prev)

    def test_neutral_pose_collision_free(self):
        pipeline = _pipeline()
        assert pipeline.check_self_collision(np.zeros(BODY_DIM)) is None

    def test_capsule_model_has_expected_pairs(self):
        pipeline = _pipeline()
        capsules = pipeline.collision_capsules(np.zeros(BODY_DIM))
        names = {c.name for c in capsules}
        assert names == {"left_forearm", "right_forearm", "torso"}


class TestPipelineOrdering:
    def test_estop_takes_priority_over_everything(self):
        pipeline = _pipeline()
        target, prev, primary = _neutral()
        target[0] = 10.0  # would trip velocity, torque and workspace stages
        secondary = primary.copy()
        secondary[0] += 1.0  # encoder fault
        decision = pipeline.check(target, prev, primary, secondary)
        assert decision.estop
        assert REASON_DUAL_ENCODER_FAULT in decision.reason

    def test_missing_secondary_skips_stage_one(self):
        pipeline = _pipeline()
        target, prev, primary = _neutral()
        decision = pipeline.check(target, prev, primary, None)
        assert decision.ok
        assert not pipeline.estop_latched
