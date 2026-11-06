"""rclpy-free execution core for ``action_executor_node``.

Contains the safety-relevant realtime logic so it is unit-testable with
numpy alone:

* :class:`DecodedChunk` / :class:`ChunkBuffer` -- receding-horizon buffering
  of ``g1_interfaces/ActionChunk`` data with replan-overlap blending and a
  fallback HOLD (impedance hold, grip force retained) during replan gaps or
  after chunk invalidation.
* :class:`HandForceController` -- 100 Hz force-position hybrid PI loop on the
  Inspire tactile arrays, including TACTILE_SLIP reaction (freeze position,
  bump force limit by 20 %).
* :class:`NavigationController` -- simple P-controller for ``navigate_to``
  subtasks (caps 0.4 m/s linear, 0.5 rad/s angular).
* Joint-vector layout constants and hand normalization helpers
  (BODY-41 / HAND-12 / FINGER-10 maps per CONVENTIONS.md).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Fixed joint-vector layout (CONVENTIONS.md)
# ---------------------------------------------------------------------------

BODY_DIM = 41
HAND_DIM = 12
ACTION_DIM = BODY_DIM + HAND_DIM  # 53
FORCE_DIM = 10
ARM_DIM = 14  # arm_sdk slice [0:14] of the body vector

MODE_FAST = 0
MODE_PRECISE = 1

LEFT_HAND_SLICE = slice(41, 47)
RIGHT_HAND_SLICE = slice(47, 53)
LEFT_FORCE_SLICE = slice(0, 5)
RIGHT_FORCE_SLICE = slice(5, 10)

#: Fingers per hand in FINGER-10 / HandCmd order (little..thumb).
FINGERS_PER_HAND = 5
#: Tactile cells per hand: 5 fingers x 3 phalanges.
TACTILE_CELLS = 15


def wrap_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _reshape_flat(
    name: str, flat: Sequence[float], rows: int, cols: int, allow_empty: bool
) -> np.ndarray:
    """Validate and reshape a row-major flattened field of an ActionChunk."""
    arr = np.asarray(flat, dtype=np.float64)
    if arr.size == 0 and allow_empty:
        return np.zeros((rows, cols), dtype=np.float64)
    if arr.size != rows * cols:
        raise ValueError(
            f"ActionChunk.{name}: expected {rows * cols} values "
            f"({rows}x{cols}), got {arr.size}"
        )
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"ActionChunk.{name}: non-finite values")
    return arr.reshape(rows, cols)


@dataclass(frozen=True)
class DecodedChunk:
    """An ActionChunk decoded into dense numpy arrays.

    Attributes:
        positions: ``[T, 53]`` joint-position targets (BODY-41 + HAND-12).
        velocities: ``[T, 53]`` joint-velocity feedforward.
        hand_force: ``[T, 10]`` fingertip force targets (FINGER-10 order, N).
        mode: ``MODE_FAST`` or ``MODE_PRECISE``.
        timestamp: chunk generation time (epoch seconds, offboard clock).
        action_entropy: raw VLA action-head entropy.
    """

    positions: np.ndarray
    velocities: np.ndarray
    hand_force: np.ndarray
    mode: int
    timestamp: float
    action_entropy: float

    @property
    def chunk_length(self) -> int:
        """Number of timesteps in the chunk."""
        return int(self.positions.shape[0])

    @staticmethod
    def from_flat(
        chunk_length: int,
        positions: Sequence[float],
        velocities: Sequence[float],
        hand_force: Sequence[float],
        mode: int = MODE_FAST,
        timestamp: float = 0.0,
        action_entropy: float = 0.0,
    ) -> "DecodedChunk":
        """Decode the flattened ActionChunk fields, validating sizes.

        ``velocities`` and ``hand_force`` may be empty (treated as zeros);
        ``positions`` must contain exactly ``chunk_length * 53`` finite
        values. Raises ``ValueError`` on any malformed field.
        """
        length = int(chunk_length)
        if length <= 0:
            raise ValueError(f"chunk_length must be positive, got {length}")
        if int(mode) not in (MODE_FAST, MODE_PRECISE):
            raise ValueError(f"unknown ActionChunk.mode {mode}")
        pos = _reshape_flat("positions", positions, length, ACTION_DIM, False)
        vel = _reshape_flat("velocities", velocities, length, ACTION_DIM, True)
        force = _reshape_flat("hand_force", hand_force, length, FORCE_DIM, True)
        return DecodedChunk(
            positions=pos,
            velocities=vel,
            hand_force=force,
            mode=int(mode),
            timestamp=float(timestamp),
            action_entropy=float(action_entropy),
        )


@dataclass(frozen=True)
class StepOutput:
    """One 50 Hz executor step produced by :class:`ChunkBuffer`.

    ``holding`` is True when the buffer has no fresh chunk row and is
    emitting the impedance-hold fallback: last safe body position targets
    with zero velocity, hand grip force retained (never released).
    """

    positions: np.ndarray  # (53,)
    velocities: np.ndarray  # (53,)
    hand_force: np.ndarray  # (10,)
    mode: int
    holding: bool


class ChunkBuffer:
    """Receding-horizon buffer over incoming ActionChunks.

    * :meth:`push` installs a new chunk. If the previous chunk still has
      unexecuted steps, the first ``replan_overlap * T_new`` steps of the new
      chunk are cross-faded with the remaining steps of the old chunk so the
      replan does not cause a discontinuity.
    * :meth:`step` pops one row per 50 Hz tick. When the chunk is exhausted
      (replan gap, typically 100-200 ms) it emits a HOLD: the last body
      position targets with zero velocity and the last hand force.
    * :meth:`invalidate` (DynaMem ``chunk_invalid``) discards everything and
      enters HOLD until a new chunk arrives.
    """

    def __init__(self, replan_overlap: float = 0.5) -> None:
        """
        Args:
            replan_overlap: fraction (clamped to [0, 1]) of the new chunk
                length blended against the remainder of the old chunk on
                replan arrival.
        """
        self._replan_overlap: float = float(min(max(replan_overlap, 0.0), 1.0))
        self._positions: Optional[np.ndarray] = None
        self._velocities: Optional[np.ndarray] = None
        self._hand_force: Optional[np.ndarray] = None
        self._mode: int = MODE_FAST
        self._index: int = 0
        self._last: Optional[StepOutput] = None
        self._invalidated: bool = False

    @property
    def replan_overlap(self) -> float:
        """Overlap fraction used when blending replans."""
        return self._replan_overlap

    @property
    def pending_steps(self) -> int:
        """Number of unexecuted steps in the active chunk."""
        if self._positions is None:
            return 0
        return max(0, int(self._positions.shape[0]) - self._index)

    @property
    def has_chunk(self) -> bool:
        """True if an active chunk with pending steps is installed."""
        return self.pending_steps > 0

    @property
    def invalidated(self) -> bool:
        """True after :meth:`invalidate` until the next valid chunk arrives."""
        return self._invalidated

    def push(self, chunk: DecodedChunk) -> None:
        """Install a new chunk, cross-fading with the remaining old steps.

        The first ``K = min(pending, T_new, round(replan_overlap * T_new))``
        rows of the new chunk are blended with the old chunk's pending rows
        using ``beta_i = (i + 1) / (K + 1)`` (strictly between 0 and 1), so
        the transition ramps smoothly from mostly-old to fully-new.
        """
        pos = chunk.positions.copy()
        vel = chunk.velocities.copy()
        force = chunk.hand_force.copy()
        t_new = pos.shape[0]

        pending = self.pending_steps
        overlap = int(round(self._replan_overlap * t_new))
        k = min(pending, t_new, overlap)
        if k > 0:
            assert self._positions is not None
            assert self._velocities is not None
            assert self._hand_force is not None
            old_pos = self._positions[self._index : self._index + k]
            old_vel = self._velocities[self._index : self._index + k]
            old_force = self._hand_force[self._index : self._index + k]
            beta = ((np.arange(k, dtype=np.float64) + 1.0) / (k + 1.0))[:, None]
            pos[:k] = (1.0 - beta) * old_pos + beta * pos[:k]
            vel[:k] = (1.0 - beta) * old_vel + beta * vel[:k]
            force[:k] = (1.0 - beta) * old_force + beta * force[:k]

        self._positions = pos
        self._velocities = vel
        self._hand_force = force
        self._mode = chunk.mode
        self._index = 0
        self._invalidated = False

    def invalidate(self) -> None:
        """Discard the active chunk and enter HOLD (grip force retained)."""
        self._positions = None
        self._velocities = None
        self._hand_force = None
        self._index = 0
        self._invalidated = True

    def step(self, fallback_positions: Optional[np.ndarray] = None) -> StepOutput:
        """Pop the next action row, or the HOLD fallback when exhausted.

        Args:
            fallback_positions: optional (53,) vector used as hold target if
                no chunk was ever executed (e.g. measured joint state).
        """
        if self._positions is not None and self._index < self._positions.shape[0]:
            assert self._velocities is not None and self._hand_force is not None
            i = self._index
            self._index += 1
            out = StepOutput(
                positions=self._positions[i].copy(),
                velocities=self._velocities[i].copy(),
                hand_force=self._hand_force[i].copy(),
                mode=self._mode,
                holding=False,
            )
            self._last = out
            return out

        if self._last is not None:
            # Impedance hold: keep last safe body targets, keep grip force.
            return StepOutput(
                positions=self._last.positions.copy(),
                velocities=np.zeros(ACTION_DIM, dtype=np.float64),
                hand_force=self._last.hand_force.copy(),
                mode=self._last.mode,
                holding=True,
            )

        if fallback_positions is not None:
            base = np.asarray(fallback_positions, dtype=np.float64).reshape(ACTION_DIM)
        else:
            base = np.zeros(ACTION_DIM, dtype=np.float64)
        return StepOutput(
            positions=base.copy(),
            velocities=np.zeros(ACTION_DIM, dtype=np.float64),
            hand_force=np.zeros(FORCE_DIM, dtype=np.float64),
            mode=MODE_FAST,
            holding=True,
        )


# ---------------------------------------------------------------------------
# Hand execution
# ---------------------------------------------------------------------------


def normalize_hand_positions(
    q_rad: np.ndarray, q_min_rad: np.ndarray, q_max_rad: np.ndarray
) -> np.ndarray:
    """Map 6 hand joint angles (rad) to normalized [0..1] open->closed.

    Uses the per-joint limit map ``(q - q_min) / (q_max - q_min)`` clipped to
    [0, 1] (Inspire convention: 0 = open, 1 = closed).
    """
    q = np.asarray(q_rad, dtype=np.float64).reshape(6)
    lo = np.asarray(q_min_rad, dtype=np.float64).reshape(6)
    hi = np.asarray(q_max_rad, dtype=np.float64).reshape(6)
    span = np.maximum(hi - lo, 1e-9)
    return np.clip((q - lo) / span, 0.0, 1.0)


@dataclass(frozen=True)
class HandControlOutput:
    """Result of one :class:`HandForceController` update.

    Attributes:
        position: (6,) normalized position command [0..1].
        force_limit: (5,) per-finger force limit to command (N); bumped by
            +20 % while a slip event is latched.
        slip_detected: True while the TACTILE_SLIP reaction is latched.
        mean_pressure: mean tactile pressure used for slip monitoring (N).
    """

    position: np.ndarray
    force_limit: np.ndarray
    slip_detected: bool
    mean_pressure: float


class HandForceController:
    """100 Hz force-position hybrid loop for one Inspire hand.

    A PI controller on the per-finger pressure error nudges the commanded
    flex positions toward the target fingertip forces. Per-finger measured
    force is the sum of the finger's 3 phalanx pressures (TACTILE-15,
    finger-major, little..thumb — aligned with HandCmd flex slots 0..4).

    TACTILE_SLIP reaction: while a grip is established (mean pressure above
    the contact threshold) a drop of the mean pressure by more than
    ``slip_drop_fraction`` (40 %) from its running peak latches a slip event:
    the position command freezes at the last output and the force limit is
    bumped by ``force_bump_fraction`` (20 %). Recovery/replanning is handled
    elsewhere; the latch clears only on :meth:`reset`.
    """

    def __init__(
        self,
        kp: float = 0.02,
        ki: float = 0.001,
        rate_hz: float = 100.0,
        slip_drop_fraction: float = 0.4,
        force_bump_fraction: float = 0.2,
        contact_threshold_n: float = 0.3,
        integral_limit: float = 5.0,
    ) -> None:
        self._kp = float(kp)
        self._ki = float(ki)
        self._dt = 1.0 / max(float(rate_hz), 1e-6)
        self._slip_drop_fraction = float(slip_drop_fraction)
        self._force_bump_fraction = float(force_bump_fraction)
        self._contact_threshold_n = float(contact_threshold_n)
        self._integral_limit = float(integral_limit)
        self._integral = np.zeros(FINGERS_PER_HAND, dtype=np.float64)
        self._peak_mean_pressure = 0.0
        self._slip_latched = False
        self._frozen_position: Optional[np.ndarray] = None
        self._frozen_force: Optional[np.ndarray] = None
        self._last_position: Optional[np.ndarray] = None

    @property
    def slip_latched(self) -> bool:
        """True while the TACTILE_SLIP freeze is active."""
        return self._slip_latched

    def reset(self) -> None:
        """Clear integrator, peak tracking and any latched slip event."""
        self._integral[:] = 0.0
        self._peak_mean_pressure = 0.0
        self._slip_latched = False
        self._frozen_position = None
        self._frozen_force = None
        self._last_position = None

    def update(
        self,
        target_position: np.ndarray,
        target_force: np.ndarray,
        tactile_pressure: np.ndarray,
    ) -> HandControlOutput:
        """Run one hybrid-control step.

        Args:
            target_position: (6,) normalized position command from the VLA.
            target_force: (5,) per-finger fingertip force targets (N).
            tactile_pressure: (15,) TACTILE-15 pressures (N).
        """
        target_position = np.asarray(target_position, dtype=np.float64).reshape(6)
        target_force = np.asarray(target_force, dtype=np.float64).reshape(
            FINGERS_PER_HAND
        )
        pressure = np.asarray(tactile_pressure, dtype=np.float64).reshape(
            TACTILE_CELLS
        )
        per_finger = pressure.reshape(FINGERS_PER_HAND, 3).sum(axis=1)
        mean_pressure = float(pressure.mean())

        grasping = bool(np.max(target_force) > 0.0)
        if grasping and mean_pressure > self._contact_threshold_n:
            self._peak_mean_pressure = max(self._peak_mean_pressure, mean_pressure)

        if (
            not self._slip_latched
            and grasping
            and self._peak_mean_pressure > self._contact_threshold_n
            and mean_pressure
            < (1.0 - self._slip_drop_fraction) * self._peak_mean_pressure
        ):
            # TACTILE_SLIP: freeze position, bump force limit by 20 %.
            self._slip_latched = True
            frozen = (
                self._last_position
                if self._last_position is not None
                else target_position
            )
            self._frozen_position = np.asarray(frozen, dtype=np.float64).copy()
            self._frozen_force = target_force * (1.0 + self._force_bump_fraction)

        if self._slip_latched:
            assert self._frozen_position is not None
            assert self._frozen_force is not None
            return HandControlOutput(
                position=self._frozen_position.copy(),
                force_limit=self._frozen_force.copy(),
                slip_detected=True,
                mean_pressure=mean_pressure,
            )

        error = target_force - per_finger
        self._integral = np.clip(
            self._integral + error * self._dt,
            -self._integral_limit,
            self._integral_limit,
        )
        adjust = self._kp * error + self._ki * self._integral
        position = target_position.copy()
        # Positive error (too little force) closes the finger further.
        position[:FINGERS_PER_HAND] = np.clip(
            position[:FINGERS_PER_HAND] + adjust, 0.0, 1.0
        )
        self._last_position = position.copy()
        return HandControlOutput(
            position=position,
            force_limit=target_force.copy(),
            slip_detected=False,
            mean_pressure=mean_pressure,
        )


# ---------------------------------------------------------------------------
# Navigation subtask P-controller
# ---------------------------------------------------------------------------


class NavigationController:
    """P-controller producing Twist commands toward a target in odom frame.

    Used while the active subtask starts with ``navigate_to``. Linear speed
    is capped at 0.4 m/s and scaled by the heading alignment; angular speed
    is capped at 0.5 rad/s.
    """

    def __init__(
        self,
        kp_linear: float = 0.8,
        kp_angular: float = 1.5,
        max_linear_m_s: float = 0.4,
        max_angular_rad_s: float = 0.5,
        goal_tolerance_m: float = 0.15,
    ) -> None:
        self._kp_linear = float(kp_linear)
        self._kp_angular = float(kp_angular)
        self._max_linear = float(max_linear_m_s)
        self._max_angular = float(max_angular_rad_s)
        self._goal_tolerance = float(goal_tolerance_m)

    def compute(
        self, x: float, y: float, yaw: float, target_x: float, target_y: float
    ) -> Tuple[float, float]:
        """Return ``(linear_x, angular_z)`` toward the target.

        Args:
            x, y, yaw: robot pose in odom frame (rad).
            target_x, target_y: target position in odom frame.
        """
        dx = float(target_x) - float(x)
        dy = float(target_y) - float(y)
        distance = math.hypot(dx, dy)
        if distance < self._goal_tolerance:
            return 0.0, 0.0
        heading_error = wrap_angle(math.atan2(dy, dx) - float(yaw))
        angular = max(
            -self._max_angular,
            min(self._max_angular, self._kp_angular * heading_error),
        )
        linear = min(self._max_linear, self._kp_linear * distance)
        # Drive forward only when roughly facing the target.
        linear *= max(0.0, math.cos(heading_error))
        return linear, angular
