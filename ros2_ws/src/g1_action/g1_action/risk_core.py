"""rclpy-free risk-estimation core for ``risk_estimator_node``.

Signals (all in [0, 1]):

* ``collision_proximity``: ``sigmoid((d0 - min_dist) / scale)`` over the
  minimum distance of depth pixels and LiDAR points inside the
  arm-workspace box.
* ``grasp_uncertainty``: ``1 - ObjectPose.confidence`` (top-1 SigLIP
  similarity from DynaMem).
* ``action_entropy``: mirrored from the latest ActionChunk.
* ``tactile_anomaly``: z-score spike detection on the mean tactile pressure
  over a 1 s window (z > 3 -> anomaly), with a floor while ContactState
  reports contacted-but-unstable fingers.

Decision (exact, per spec)::

    trigger_precise = (collision_proximity > 0.7)
                      OR (grasp_uncertainty > 0.6)
                      OR (action_entropy > 0.4)
                      OR (tactile_anomaly > 0.3 AND phase == GRASP)

Tactile VLA extension overrides:
* slip_detected True        -> trigger_precise = True regardless;
* ContactState.grasp_stable AND collision/uncertainty/entropy all below
  their thresholds -> trigger_precise = False (a confirmed stable grasp
  returns to the fast path even if the anomaly score is elevated).
"""

from __future__ import annotations

import math
from collections import deque
from enum import IntEnum
from typing import Deque, Sequence

import numpy as np


class TaskPhase(IntEnum):
    """Task phase parsed from the planner subtask (navigation < approach < grasp)."""

    UNKNOWN = 0
    NAVIGATION = 1
    APPROACH = 2
    GRASP = 3
    OTHER = 4


def phase_from_subtask(text: str) -> TaskPhase:
    """Map a SubtaskToken text to a :class:`TaskPhase`.

    Grasp-phase detection follows CONVENTIONS.md: text starts with ``grasp``
    (covers grasp, grasp_bimanual, grasp_precision).
    """
    token = (text or "").strip().lower()
    if not token:
        return TaskPhase.UNKNOWN
    if token.startswith("navigate_to"):
        return TaskPhase.NAVIGATION
    if token.startswith("approach"):
        return TaskPhase.APPROACH
    if token.startswith("grasp"):
        return TaskPhase.GRASP
    return TaskPhase.OTHER


def sigmoid(x: float) -> float:
    """Numerically stable logistic function."""
    if x >= 0.0:
        z = math.exp(-x)
        return 1.0 / (1.0 + z)
    z = math.exp(x)
    return z / (1.0 + z)


def collision_proximity(
    min_distance_m: float, d0: float = 0.35, scale: float = 0.1
) -> float:
    """``sigmoid((d0 - min_dist) / scale)``; 0.5 at ``d0``, ->1 when close."""
    if not math.isfinite(min_distance_m):
        return 0.0
    return sigmoid((d0 - float(min_distance_m)) / max(float(scale), 1e-9))


def min_depth_distance(
    depth: np.ndarray,
    stride: int = 8,
    near_clip_m: float = 0.05,
    far_clip_m: float = 0.8,
) -> float:
    """Minimum valid depth (m) after subsampling, or +inf if none.

    Depth pixels are treated as forward-range measurements; pixels beyond
    ``far_clip_m`` (arm-workspace forward extent) or closer than
    ``near_clip_m`` (sensor noise floor) are ignored. This is a documented
    approximation: without camera extrinsics at this level, the workspace
    box membership of a depth pixel is approximated by its range.
    """
    arr = np.asarray(depth, dtype=np.float64)
    if arr.ndim == 2:
        arr = arr[:: max(int(stride), 1), :: max(int(stride), 1)]
    else:
        arr = arr.reshape(-1)[:: max(int(stride), 1)]
    valid = arr[np.isfinite(arr) & (arr > near_clip_m) & (arr <= far_clip_m)]
    if valid.size == 0:
        return math.inf
    return float(valid.min())


def min_point_distance(
    points_xyz: np.ndarray, workspace_bounds: Sequence[float]
) -> float:
    """Minimum norm of LiDAR points inside the workspace box, or +inf.

    Args:
        points_xyz: (N, 3) points in the base frame.
        workspace_bounds: ``[x_min, x_max, y_min, y_max, z_min, z_max]``.
    """
    pts = np.asarray(points_xyz, dtype=np.float64)
    if pts.size == 0:
        return math.inf
    pts = pts.reshape(-1, 3)
    pts = pts[np.all(np.isfinite(pts), axis=1)]
    if pts.size == 0:
        return math.inf
    b = np.asarray(workspace_bounds, dtype=np.float64).reshape(6)
    inside = (
        (pts[:, 0] >= b[0])
        & (pts[:, 0] <= b[1])
        & (pts[:, 1] >= b[2])
        & (pts[:, 1] <= b[3])
        & (pts[:, 2] >= b[4])
        & (pts[:, 2] <= b[5])
    )
    pts = pts[inside]
    if pts.shape[0] == 0:
        return math.inf
    return float(np.linalg.norm(pts, axis=1).min())


class TactileAnomalyDetector:
    """Z-score spike detector on the mean tactile pressure.

    Keeps a sliding window (1 s of samples); each update computes the
    z-score of the new sample against the window statistics *before*
    appending it. Scores at or below ``z_threshold`` map to anomaly 0;
    above, anomaly ramps ``(z - z_threshold) / z_threshold`` clipped to 1.

    The window std is floored at ``min_std`` (absolute pressure noise floor
    in newtons) so ordinary sensor noise on a near-flat signal cannot
    register as a spike.
    """

    def __init__(
        self,
        window_size: int = 10,
        z_threshold: float = 3.0,
        min_samples: int = 5,
        min_std: float = 0.05,
    ) -> None:
        """
        Args:
            window_size: samples in the window (rate * 1 s).
            z_threshold: z-score above which a spike counts as anomaly.
            min_samples: warm-up sample count before scoring.
            min_std: std floor (N) — the tactile sensor noise floor.
        """
        self._window: Deque[float] = deque(maxlen=max(2, int(window_size)))
        self._z_threshold = float(z_threshold)
        self._min_samples = max(2, int(min_samples))
        self._min_std = max(float(min_std), 1e-9)

    def reset(self) -> None:
        """Drop all history."""
        self._window.clear()

    def update(self, mean_pressure: float) -> float:
        """Push one sample and return the anomaly score in [0, 1]."""
        x = float(mean_pressure)
        if len(self._window) < self._min_samples:
            self._window.append(x)
            return 0.0
        window = np.asarray(self._window, dtype=np.float64)
        mu = float(window.mean())
        sd = max(float(window.std()), self._min_std)
        z = abs(x - mu) / sd
        self._window.append(x)
        if z <= self._z_threshold:
            return 0.0
        return min(1.0, (z - self._z_threshold) / self._z_threshold)


def decide_trigger_precise(
    collision_proximity: float,
    grasp_uncertainty: float,
    action_entropy: float,
    tactile_anomaly: float,
    phase: TaskPhase,
    slip_detected: bool = False,
    grasp_stable: bool = False,
    collision_threshold: float = 0.7,
    uncertainty_threshold: float = 0.6,
    entropy_threshold: float = 0.4,
    tactile_threshold: float = 0.3,
) -> bool:
    """Fast <-> precise dispatch decision (see module docstring for the rule).

    ``slip_detected`` forces True regardless of every other signal.
    ``grasp_stable`` forces False when collision proximity, grasp
    uncertainty and action entropy are all at/below their thresholds
    (the stable-grasp tactile evidence overrides the anomaly score).
    """
    if slip_detected:
        return True
    others_below = (
        collision_proximity <= collision_threshold
        and grasp_uncertainty <= uncertainty_threshold
        and action_entropy <= entropy_threshold
    )
    if grasp_stable and others_below:
        return False
    return (
        collision_proximity > collision_threshold
        or grasp_uncertainty > uncertainty_threshold
        or action_entropy > entropy_threshold
        or (tactile_anomaly > tactile_threshold and phase == TaskPhase.GRASP)
    )
