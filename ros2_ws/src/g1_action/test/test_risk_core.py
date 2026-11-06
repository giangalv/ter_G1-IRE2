"""Risk score with all U6 signals (numpy only, no rclpy)."""

import numpy as np
import pytest

from g1_action.risk_core import (
    TactileAnomalyDetector,
    TaskPhase,
    collision_proximity,
    decide_trigger_precise,
    min_depth_distance,
    min_point_distance,
    phase_from_subtask,
)


def _decide(**overrides) -> bool:
    base = dict(
        collision_proximity=0.0,
        grasp_uncertainty=0.0,
        action_entropy=0.0,
        tactile_anomaly=0.0,
        phase=TaskPhase.APPROACH,
    )
    base.update(overrides)
    return decide_trigger_precise(**base)


class TestThresholds:
    def test_all_low_stays_fast(self):
        assert not _decide()

    def test_collision_proximity_fires_alone(self):
        assert not _decide(collision_proximity=0.7)  # at threshold: no
        assert _decide(collision_proximity=0.71)

    def test_grasp_uncertainty_fires_alone(self):
        assert not _decide(grasp_uncertainty=0.6)
        assert _decide(grasp_uncertainty=0.61)

    def test_action_entropy_fires_alone(self):
        assert not _decide(action_entropy=0.4)
        assert _decide(action_entropy=0.41)

    def test_tactile_anomaly_fires_only_in_grasp_phase(self):
        # Elevated anomaly outside grasp: no trigger.
        for phase in (
            TaskPhase.UNKNOWN,
            TaskPhase.NAVIGATION,
            TaskPhase.APPROACH,
            TaskPhase.OTHER,
        ):
            assert not _decide(tactile_anomaly=0.9, phase=phase)
        # Same anomaly during grasp: trigger.
        assert _decide(tactile_anomaly=0.31, phase=TaskPhase.GRASP)
        assert not _decide(tactile_anomaly=0.3, phase=TaskPhase.GRASP)


class TestTactileOverrides:
    def test_slip_forces_precise_regardless(self):
        assert _decide(slip_detected=True)
        assert _decide(slip_detected=True, phase=TaskPhase.NAVIGATION)
        # Even a stable grasp cannot override slip.
        assert _decide(slip_detected=True, grasp_stable=True)

    def test_grasp_stable_returns_to_fast_path(self):
        # Anomaly alone would trigger during grasp...
        assert _decide(tactile_anomaly=0.9, phase=TaskPhase.GRASP)
        # ...but a confirmed stable grasp with all other signals below
        # thresholds returns to the fast path.
        assert not _decide(
            tactile_anomaly=0.9, phase=TaskPhase.GRASP, grasp_stable=True
        )

    def test_grasp_stable_does_not_mask_other_signals(self):
        assert _decide(
            collision_proximity=0.8, grasp_stable=True, phase=TaskPhase.GRASP
        )
        assert _decide(
            grasp_uncertainty=0.7, grasp_stable=True, phase=TaskPhase.GRASP
        )
        assert _decide(
            action_entropy=0.5, grasp_stable=True, phase=TaskPhase.GRASP
        )


class TestPhaseParsing:
    def test_vocabulary(self):
        assert phase_from_subtask("navigate_to kitchen") == TaskPhase.NAVIGATION
        assert phase_from_subtask("approach red mug") == TaskPhase.APPROACH
        assert phase_from_subtask("grasp red mug") == TaskPhase.GRASP
        assert phase_from_subtask("grasp_bimanual box") == TaskPhase.GRASP
        assert phase_from_subtask("grasp_precision red mug") == TaskPhase.GRASP
        assert phase_from_subtask("place table") == TaskPhase.OTHER
        assert phase_from_subtask("") == TaskPhase.UNKNOWN

    def test_ordering_navigation_approach_grasp(self):
        assert TaskPhase.NAVIGATION < TaskPhase.APPROACH < TaskPhase.GRASP


class TestCollisionProximity:
    def test_sigmoid_shape(self):
        assert collision_proximity(0.35) == pytest.approx(0.5)
        assert collision_proximity(0.05) > 0.9
        assert collision_proximity(1.0) < 0.01
        assert collision_proximity(float("inf")) == 0.0

    def test_close_obstacle_crosses_threshold(self):
        assert collision_proximity(0.2) > 0.7  # inside d0 - scale region


class TestMinimumDistances:
    def test_depth_min_with_subsampling(self):
        depth = np.full((48, 64), 5.0)
        depth[8, 8] = 0.42  # on the stride-8 grid
        assert min_depth_distance(depth, stride=8) == pytest.approx(0.42)

    def test_depth_ignores_out_of_range(self):
        depth = np.array([[0.0, 0.01, 3.0, np.nan, np.inf]])
        assert min_depth_distance(depth, stride=1) == float("inf")

    def test_lidar_points_filtered_by_workspace_box(self):
        bounds = [-0.2, 0.8, -0.6, 0.6, 0.0, 1.6]
        points = np.array(
            [
                [0.3, 0.0, 0.5],  # inside, norm ~0.583
                [5.0, 0.0, 0.5],  # outside x
                [0.3, 0.0, 2.0],  # outside z
            ]
        )
        d = min_point_distance(points, bounds)
        assert d == pytest.approx(np.linalg.norm([0.3, 0.0, 0.5]))

    def test_lidar_empty_returns_inf(self):
        assert min_point_distance(np.zeros((0, 3)), [-1, 1, -1, 1, -1, 1]) == float(
            "inf"
        )


class TestTactileAnomalyDetector:
    def test_spike_detected_after_stable_window(self):
        det = TactileAnomalyDetector(window_size=10, z_threshold=3.0)
        rng = np.random.default_rng(3)
        for _ in range(10):
            det.update(1.0 + float(rng.normal(0.0, 0.01)))
        anomaly = det.update(5.0)  # massive spike
        assert anomaly > 0.9

    def test_stable_signal_no_anomaly(self):
        det = TactileAnomalyDetector(window_size=10, z_threshold=3.0)
        rng = np.random.default_rng(4)
        scores = [
            det.update(2.0 + float(rng.normal(0.0, 0.05))) for _ in range(50)
        ]
        assert max(scores) < 0.3

    def test_warmup_returns_zero(self):
        det = TactileAnomalyDetector(window_size=10, min_samples=5)
        assert det.update(100.0) == 0.0
        assert det.update(0.0) == 0.0

    def test_reset_clears_history(self):
        det = TactileAnomalyDetector(window_size=10)
        for _ in range(10):
            det.update(1.0)
        det.reset()
        assert det.update(50.0) == 0.0  # warming up again
