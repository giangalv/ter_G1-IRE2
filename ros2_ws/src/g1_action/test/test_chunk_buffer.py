"""Tests for the receding-horizon ChunkBuffer and the hand hybrid loop
(numpy only, no rclpy)."""

import numpy as np
import pytest

from g1_action.executor_core import (
    ACTION_DIM,
    FORCE_DIM,
    MODE_FAST,
    MODE_PRECISE,
    ChunkBuffer,
    DecodedChunk,
    HandForceController,
    NavigationController,
)


def _chunk(row_values, mode=MODE_FAST, force_values=None):
    """Chunk whose row t is constant `row_values[t]` (positions and force)."""
    length = len(row_values)
    positions = np.repeat(
        np.asarray(row_values, dtype=float)[:, None], ACTION_DIM, axis=1
    )
    if force_values is None:
        force_values = row_values
    force = np.repeat(
        np.asarray(force_values, dtype=float)[:, None], FORCE_DIM, axis=1
    )
    return DecodedChunk.from_flat(
        length,
        positions.ravel(),
        np.zeros(length * ACTION_DIM),
        force.ravel(),
        mode=mode,
    )


class TestRecedingHorizon:
    def test_steps_pop_rows_in_order(self):
        buf = ChunkBuffer(replan_overlap=0.5)
        buf.push(_chunk([1.0, 2.0, 3.0, 4.0]))
        values = [buf.step().positions[0] for _ in range(4)]
        assert values == [1.0, 2.0, 3.0, 4.0]

    def test_mode_and_flags_carried_through(self):
        buf = ChunkBuffer()
        buf.push(_chunk([5.0], mode=MODE_PRECISE))
        out = buf.step()
        assert out.mode == MODE_PRECISE
        assert not out.holding

    def test_exhausted_chunk_holds_last_row(self):
        buf = ChunkBuffer()
        buf.push(_chunk([1.0, 2.0]))
        buf.step()
        buf.step()
        hold = buf.step()  # replan gap
        assert hold.holding
        assert hold.positions[0] == pytest.approx(2.0)
        assert np.all(hold.velocities == 0.0)  # impedance hold
        assert hold.hand_force[0] == pytest.approx(2.0)  # grip force kept

    def test_malformed_chunk_rejected(self):
        with pytest.raises(ValueError):
            DecodedChunk.from_flat(2, np.zeros(3), np.zeros(0), np.zeros(0))
        with pytest.raises(ValueError):
            DecodedChunk.from_flat(0, np.zeros(0), np.zeros(0), np.zeros(0))


class TestReplanOverlapBlend:
    def test_overlap_blend_on_replan(self):
        buf = ChunkBuffer(replan_overlap=0.5)
        buf.push(_chunk([0.0] * 6))
        buf.step()
        buf.step()  # 4 steps of the old chunk remain

        buf.push(_chunk([1.0] * 6))  # K = min(4, 6, 0.5*6) = 3 overlap steps
        values = [buf.step().positions[0] for _ in range(6)]
        # betas (i+1)/(K+1): 0.25, 0.5, 0.75, then fully-new rows.
        assert values[:3] == pytest.approx([0.25, 0.5, 0.75])
        assert values[3:] == pytest.approx([1.0, 1.0, 1.0])
        # Midpoint of the overlap is the average of old and new.
        assert values[1] == pytest.approx(0.5)

    def test_replan_with_exhausted_chunk_has_no_overlap(self):
        buf = ChunkBuffer(replan_overlap=0.5)
        buf.push(_chunk([0.0]))
        buf.step()
        buf.push(_chunk([1.0, 1.0]))
        assert buf.step().positions[0] == pytest.approx(1.0)

    def test_zero_overlap_disables_blend(self):
        buf = ChunkBuffer(replan_overlap=0.0)
        buf.push(_chunk([0.0] * 4))
        buf.step()
        buf.push(_chunk([1.0] * 4))
        assert buf.step().positions[0] == pytest.approx(1.0)

    def test_hand_force_blended_too(self):
        buf = ChunkBuffer(replan_overlap=0.5)
        buf.push(_chunk([0.0] * 6, force_values=[2.0] * 6))
        buf.step()
        buf.push(_chunk([1.0] * 6, force_values=[4.0] * 6))
        out = buf.step()
        assert out.hand_force[0] == pytest.approx(2.0 + 0.25 * 2.0)


class TestHoldOnInvalidation:
    def test_invalidation_discards_and_holds(self):
        buf = ChunkBuffer(replan_overlap=0.5)
        buf.push(_chunk([0.3, 0.3, 0.3], force_values=[2.5, 2.5, 2.5]))
        buf.step()

        buf.invalidate()
        assert buf.invalidated
        assert not buf.has_chunk

        for _ in range(3):
            hold = buf.step()
            assert hold.holding
            # Body holds the last safe targets.
            assert np.allclose(hold.positions, 0.3)
            assert np.all(hold.velocities == 0.0)
            # Hand grip force is held, NOT released.
            assert np.allclose(hold.hand_force, 2.5)

    def test_new_chunk_after_invalidation_resumes(self):
        buf = ChunkBuffer(replan_overlap=0.5)
        buf.push(_chunk([0.3]))
        buf.step()
        buf.invalidate()
        buf.push(_chunk([0.9]))
        out = buf.step()
        assert not out.holding
        assert not buf.invalidated
        # No overlap blend against a discarded chunk.
        assert out.positions[0] == pytest.approx(0.9)

    def test_hold_without_any_chunk_uses_fallback(self):
        buf = ChunkBuffer()
        fallback = np.full(ACTION_DIM, 0.7)
        out = buf.step(fallback_positions=fallback)
        assert out.holding
        assert np.allclose(out.positions, 0.7)
        assert np.all(out.hand_force == 0.0)


class TestHandForceController:
    def test_pi_closes_toward_target_force(self):
        ctrl = HandForceController(kp=0.02, ki=0.001, rate_hz=100.0)
        target_pos = np.full(6, 0.5)
        target_force = np.full(5, 5.0)
        out = ctrl.update(target_pos, target_force, np.zeros(15))
        # No measured pressure: fingers close beyond the commanded position.
        assert np.all(out.position[:5] > 0.5)
        assert out.position[5] == pytest.approx(0.5)  # thumb rotation untouched

    def test_slip_freezes_position_and_bumps_force(self):
        ctrl = HandForceController(kp=0.02, ki=0.001, rate_hz=100.0)
        target_pos = np.full(6, 0.7)
        target_force = np.full(5, 5.0)
        steady = np.full(15, 5.0 / 3.0)  # per-finger force = target -> no error
        for _ in range(20):
            out = ctrl.update(target_pos, target_force, steady)
        assert not out.slip_detected
        frozen_reference = out.position.copy()

        # Pressure drops 60 % (> 40 %) mid-grasp -> TACTILE_SLIP.
        slipped = ctrl.update(target_pos, target_force, steady * 0.4)
        assert slipped.slip_detected
        assert np.allclose(slipped.position, frozen_reference)
        assert np.allclose(slipped.force_limit, target_force * 1.2)

        # Latch persists even if pressure recovers, until reset().
        after = ctrl.update(target_pos, target_force, steady)
        assert after.slip_detected
        assert np.allclose(after.position, frozen_reference)
        ctrl.reset()
        clean = ctrl.update(target_pos, target_force, steady)
        assert not clean.slip_detected

    def test_no_slip_without_established_grip(self):
        ctrl = HandForceController()
        target_pos = np.full(6, 0.2)
        # Zero target force (no grasp intent): pressure drops are not slip.
        out = ctrl.update(target_pos, np.zeros(5), np.full(15, 2.0))
        out = ctrl.update(target_pos, np.zeros(5), np.zeros(15))
        assert not out.slip_detected


class TestNavigationController:
    def test_caps_and_goal_tolerance(self):
        nav = NavigationController(
            max_linear_m_s=0.4, max_angular_rad_s=0.5, goal_tolerance_m=0.15
        )
        linear, angular = nav.compute(0.0, 0.0, 0.0, 10.0, 0.0)
        assert linear == pytest.approx(0.4)
        assert angular == pytest.approx(0.0)

        linear, angular = nav.compute(0.0, 0.0, 0.0, 0.0, 10.0)
        assert abs(angular) == pytest.approx(0.5)
        assert linear <= 0.4 + 1e-9

        assert nav.compute(0.0, 0.0, 0.0, 0.05, 0.0) == (0.0, 0.0)

    def test_does_not_drive_backward(self):
        nav = NavigationController()
        linear, _ = nav.compute(0.0, 0.0, 0.0, -5.0, 0.0)  # target behind
        assert linear == pytest.approx(0.0)
