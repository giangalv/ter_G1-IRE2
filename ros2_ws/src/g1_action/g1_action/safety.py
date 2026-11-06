"""rclpy-free safety pipeline for ``action_executor_node``.

Implements the mandated five-stage pipeline (CONVENTIONS.md), each stage
producing a :class:`SafetyDecision`:

1. dual-encoder fault check     -> latched ESTOP
2. joint velocity limit         -> per-joint delta clamp
3. joint torque limit (120 N*m) -> clamp via stiffness model
4. workspace boundary check     -> BLOCK (hold)
5. self-collision capsule check -> BLOCK (hold)
6. execute

The workspace check uses a 3-segment planar-chain forward-kinematics
approximation (torso offset + upper arm + forearm) for each wrist; the
self-collision check uses capsules around both forearms and the torso with
the standard clamped closest-point segment-segment distance.

Kinematic approximation (documented, not exact G1 kinematics):
* the base frame sits at the base footprint, z up, x forward;
* shoulders at ``(0, +/-shoulder_offset_y_m, shoulder_offset_z_m)``;
* shoulder_pitch and elbow_pitch rotate the chain in the sagittal plane
  (positive pitch = forward reach), shoulder_roll rotates the whole arm
  about the x axis (positive roll moves the arm toward +y);
* shoulder_yaw and the wrist joints are ignored by the approximation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from g1_action.executor_core import ACTION_DIM, BODY_DIM

# Decision reason codes.
REASON_OK = "OK"
REASON_DUAL_ENCODER_FAULT = "DUAL_ENCODER_FAULT"
REASON_ESTOP_LATCHED = "ESTOP_LATCHED"
REASON_VELOCITY_CLAMPED = "VELOCITY_CLAMPED"
REASON_TORQUE_CLAMPED = "TORQUE_CLAMPED"
REASON_WORKSPACE_BLOCK = "WORKSPACE_BLOCK"
REASON_SELF_COLLISION_BLOCK = "SELF_COLLISION_BLOCK"

_LEFT_ARM_BASE = 0
_RIGHT_ARM_BASE = 7
_IDX_SHOULDER_PITCH = 0
_IDX_SHOULDER_ROLL = 1
_IDX_ELBOW_PITCH = 3


def segment_segment_distance(
    p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray
) -> float:
    """Minimum distance between segments ``[p1, q1]`` and ``[p2, q2]``.

    Standard clamped closest-point algorithm (Ericson, "Real-Time Collision
    Detection", 5.1.9). Handles all degenerate cases: either or both
    segments reduced to points, parallel and collinear segments.
    """
    p1 = np.asarray(p1, dtype=np.float64).reshape(3)
    q1 = np.asarray(q1, dtype=np.float64).reshape(3)
    p2 = np.asarray(p2, dtype=np.float64).reshape(3)
    q2 = np.asarray(q2, dtype=np.float64).reshape(3)

    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = float(np.dot(d1, d1))
    e = float(np.dot(d2, d2))
    f = float(np.dot(d2, r))
    eps = 1e-12

    if a <= eps and e <= eps:
        # Both segments are points.
        return float(np.linalg.norm(r))
    if a <= eps:
        # First segment is a point.
        s = 0.0
        t = min(max(f / e, 0.0), 1.0)
    else:
        c = float(np.dot(d1, r))
        if e <= eps:
            # Second segment is a point.
            t = 0.0
            s = min(max(-c / a, 0.0), 1.0)
        else:
            b = float(np.dot(d1, d2))
            denom = a * e - b * b
            if denom > eps:
                s = min(max((b * f - c * e) / denom, 0.0), 1.0)
            else:
                # Parallel segments: pick an arbitrary s and clamp t.
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = min(max(-c / a, 0.0), 1.0)
            elif t > 1.0:
                t = 1.0
                s = min(max((b - c) / a, 0.0), 1.0)
    closest1 = p1 + d1 * s
    closest2 = p2 + d2 * t
    return float(np.linalg.norm(closest1 - closest2))


@dataclass
class SafetyDecision:
    """Outcome of the safety pipeline for one 50 Hz tick.

    Attributes:
        ok: True if the (possibly clamped) action may be executed.
        action: the action to command; on BLOCK/ESTOP this is the hold
            action (previous safe command).
        reason: reason code(s); ``OK`` or a comma-joined list such as
            ``VELOCITY_CLAMPED,TORQUE_CLAMPED``.
        estop: True when the decision demands a latched emergency stop.
        blocked: True when a boundary/collision stage blocked the action.
    """

    ok: bool
    action: np.ndarray
    reason: str
    estop: bool = False
    blocked: bool = False


@dataclass(frozen=True)
class Capsule:
    """A capsule: segment from ``p0`` to ``p1`` with ``radius``."""

    p0: np.ndarray
    p1: np.ndarray
    radius: float
    name: str = field(default="capsule")


class SafetyPipeline:
    """Stateful five-stage safety pipeline (ESTOP latch across calls)."""

    def __init__(
        self,
        rate_hz: float = 50.0,
        max_joint_velocity_rad_s: float = 2.5,
        max_joint_torque_nm: float = 120.0,
        dual_encoder_fault_threshold: float = 0.05,
        workspace_bounds: Sequence[float] = (-0.2, 0.8, -0.6, 0.6, 0.0, 1.6),
        stiffness_nm_per_rad: float = 80.0,
        upper_arm_m: float = 0.28,
        forearm_m: float = 0.26,
        shoulder_offset_y_m: float = 0.15,
        shoulder_offset_z_m: float = 1.1,
        torso_capsule_z_min_m: float = 0.7,
        torso_capsule_radius_m: float = 0.07,
        forearm_capsule_radius_m: float = 0.035,
        self_collision_margin_m: float = 0.01,
    ) -> None:
        """
        Args:
            rate_hz: executor tick rate (velocity clamp uses ``v_max / rate``).
            max_joint_velocity_rad_s: per-joint velocity limit (U6: 2.5).
            max_joint_torque_nm: per-joint torque limit (U6 knee spec: 120).
            dual_encoder_fault_threshold: max |primary - secondary| (rad).
            workspace_bounds: ``[x_min, x_max, y_min, y_max, z_min, z_max]``
                around the base for the wrist positions.
            stiffness_nm_per_rad: joint stiffness used to estimate torque
                demand from tracking error.
            upper_arm_m / forearm_m / shoulder_offset_*: FK approximation
                segment lengths and torso offset.
            torso_capsule_z_min_m / *_radius_m / self_collision_margin_m:
                capsule model for the self-collision check.
        """
        bounds = np.asarray(workspace_bounds, dtype=np.float64).reshape(6)
        if bounds[0] >= bounds[1] or bounds[2] >= bounds[3] or bounds[4] >= bounds[5]:
            raise ValueError(f"degenerate workspace_bounds {bounds.tolist()}")
        self.rate_hz = float(rate_hz)
        self.max_joint_velocity_rad_s = float(max_joint_velocity_rad_s)
        self.max_joint_torque_nm = float(max_joint_torque_nm)
        self.dual_encoder_fault_threshold = float(dual_encoder_fault_threshold)
        self.workspace_bounds = bounds
        self.stiffness_nm_per_rad = float(stiffness_nm_per_rad)
        self.upper_arm_m = float(upper_arm_m)
        self.forearm_m = float(forearm_m)
        self.shoulder_offset_y_m = float(shoulder_offset_y_m)
        self.shoulder_offset_z_m = float(shoulder_offset_z_m)
        self.torso_capsule_z_min_m = float(torso_capsule_z_min_m)
        self.torso_capsule_radius_m = float(torso_capsule_radius_m)
        self.forearm_capsule_radius_m = float(forearm_capsule_radius_m)
        self.self_collision_margin_m = float(self_collision_margin_m)
        self._estop_latched = False

    # -- state ------------------------------------------------------------

    @property
    def estop_latched(self) -> bool:
        """True once a dual-encoder fault has been detected, until reset."""
        return self._estop_latched

    def reset(self) -> None:
        """Clear the latched ESTOP (explicit operator action only)."""
        self._estop_latched = False

    # -- stage helpers (public for unit tests and diagnostics) -------------

    @staticmethod
    def encoder_delta(
        primary: np.ndarray, secondary: np.ndarray
    ) -> np.ndarray:
        """|primary - secondary| per body joint (41,)."""
        p = np.asarray(primary, dtype=np.float64).reshape(BODY_DIM)
        s = np.asarray(secondary, dtype=np.float64).reshape(BODY_DIM)
        return np.abs(p - s)

    def clamp_velocity(
        self, target: np.ndarray, previous: np.ndarray
    ) -> Tuple[np.ndarray, bool]:
        """Clamp per-joint body deltas to ``max_velocity / rate`` rad/tick."""
        out = np.asarray(target, dtype=np.float64).copy()
        prev = np.asarray(previous, dtype=np.float64)
        max_delta = self.max_joint_velocity_rad_s / self.rate_hz
        delta = out[:BODY_DIM] - prev[:BODY_DIM]
        clipped = np.clip(delta, -max_delta, max_delta)
        changed = bool(np.any(np.abs(delta - clipped) > 1e-12))
        out[:BODY_DIM] = prev[:BODY_DIM] + clipped
        return out, changed

    def clamp_torque(
        self, target: np.ndarray, measured: np.ndarray
    ) -> Tuple[np.ndarray, bool]:
        """Clamp targets so estimated torque stays within the limit.

        Torque demand is estimated as ``stiffness * (target - measured)``
        per body joint; the target is clamped so the estimate never exceeds
        ``max_joint_torque_nm`` (120 N*m knee spec).
        """
        out = np.asarray(target, dtype=np.float64).copy()
        meas = np.asarray(measured, dtype=np.float64).reshape(BODY_DIM)
        max_err = self.max_joint_torque_nm / max(self.stiffness_nm_per_rad, 1e-9)
        err = out[:BODY_DIM] - meas
        clipped = np.clip(err, -max_err, max_err)
        changed = bool(np.any(np.abs(err - clipped) > 1e-12))
        out[:BODY_DIM] = meas + clipped
        return out, changed

    def wrist_positions(self, body: np.ndarray) -> np.ndarray:
        """(2, 3) approximate left/right wrist positions in the base frame."""
        geometry = self._arm_geometry(body)
        return np.stack([geometry[0][2], geometry[1][2]])

    def check_workspace(self, body: np.ndarray) -> Tuple[bool, np.ndarray]:
        """Check both wrists against ``workspace_bounds``.

        Returns:
            ``(ok, wrists)`` where ``wrists`` is the (2, 3) FK result.
        """
        wrists = self.wrist_positions(body)
        b = self.workspace_bounds
        ok = bool(
            np.all(
                (wrists[:, 0] >= b[0])
                & (wrists[:, 0] <= b[1])
                & (wrists[:, 1] >= b[2])
                & (wrists[:, 1] <= b[3])
                & (wrists[:, 2] >= b[4])
                & (wrists[:, 2] <= b[5])
            )
        )
        return ok, wrists

    def collision_capsules(self, body: np.ndarray) -> List[Capsule]:
        """Capsules for the self-collision model: both forearms + torso."""
        geometry = self._arm_geometry(body)
        capsules = [
            Capsule(
                p0=geometry[0][1],
                p1=geometry[0][2],
                radius=self.forearm_capsule_radius_m,
                name="left_forearm",
            ),
            Capsule(
                p0=geometry[1][1],
                p1=geometry[1][2],
                radius=self.forearm_capsule_radius_m,
                name="right_forearm",
            ),
            Capsule(
                p0=np.array([0.0, 0.0, self.torso_capsule_z_min_m]),
                p1=np.array([0.0, 0.0, self.shoulder_offset_z_m]),
                radius=self.torso_capsule_radius_m,
                name="torso",
            ),
        ]
        return capsules

    def check_self_collision(self, body: np.ndarray) -> Optional[str]:
        """Return a description of the first colliding capsule pair, or None."""
        capsules = self.collision_capsules(body)
        pairs = ((0, 2), (1, 2), (0, 1))  # L-torso, R-torso, L-R
        for i, j in pairs:
            ci, cj = capsules[i], capsules[j]
            dist = segment_segment_distance(ci.p0, ci.p1, cj.p0, cj.p1)
            threshold = ci.radius + cj.radius + self.self_collision_margin_m
            if dist < threshold:
                return f"{ci.name}-{cj.name} distance {dist:.3f} < {threshold:.3f}"
        return None

    # -- full pipeline ------------------------------------------------------

    def check(
        self,
        target: np.ndarray,
        prev_cmd: np.ndarray,
        primary: Optional[np.ndarray],
        secondary: Optional[np.ndarray],
    ) -> SafetyDecision:
        """Run the five-stage pipeline on one candidate action.

        Args:
            target: candidate (53,) action for this tick.
            prev_cmd: previously commanded (53,) action (hold reference).
            primary: latest primary-encoder body positions (41,), or None.
            secondary: latest secondary-encoder body positions (41,), or None
                (stage 1 is skipped until both encoder streams are known).

        Returns:
            SafetyDecision. ``estop`` decisions are latched: every later
            call returns an ESTOP decision until :meth:`reset`.
        """
        target = np.asarray(target, dtype=np.float64).reshape(ACTION_DIM)
        prev = np.asarray(prev_cmd, dtype=np.float64).reshape(ACTION_DIM)
        hold = prev.copy()

        # Stage 1: dual-encoder fault -> ESTOP (latched).
        if self._estop_latched:
            return SafetyDecision(
                ok=False, action=hold, reason=REASON_ESTOP_LATCHED, estop=True
            )
        if primary is not None and secondary is not None:
            delta = self.encoder_delta(primary, secondary)
            if bool(np.any(delta > self.dual_encoder_fault_threshold)):
                self._estop_latched = True
                return SafetyDecision(
                    ok=False,
                    action=hold,
                    reason=REASON_DUAL_ENCODER_FAULT,
                    estop=True,
                )

        reasons: List[str] = []

        # Stage 2: joint velocity clamp.
        action, changed = self.clamp_velocity(target, prev)
        if changed:
            reasons.append(REASON_VELOCITY_CLAMPED)

        # Stage 3: joint torque clamp (120 N*m knee spec).
        if primary is not None:
            action, changed = self.clamp_torque(action, primary)
            if changed:
                reasons.append(REASON_TORQUE_CLAMPED)

        # Stage 4: workspace boundary check -> BLOCK (hold).
        ok, wrists = self.check_workspace(action[:BODY_DIM])
        if not ok:
            return SafetyDecision(
                ok=False,
                action=hold,
                reason=(
                    f"{REASON_WORKSPACE_BLOCK}: wrists "
                    f"{np.round(wrists, 3).tolist()} outside "
                    f"{self.workspace_bounds.tolist()}"
                ),
                blocked=True,
            )

        # Stage 5: self-collision capsule check -> BLOCK (hold).
        collision = self.check_self_collision(action[:BODY_DIM])
        if collision is not None:
            return SafetyDecision(
                ok=False,
                action=hold,
                reason=f"{REASON_SELF_COLLISION_BLOCK}: {collision}",
                blocked=True,
            )

        # Stage 6: execute.
        return SafetyDecision(
            ok=True,
            action=action,
            reason=",".join(reasons) if reasons else REASON_OK,
        )

    # -- forward-kinematics approximation -----------------------------------

    def _arm_geometry(
        self, body: np.ndarray
    ) -> List[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """(shoulder, elbow, wrist) points for the left and right arms.

        3-segment planar-chain approximation: torso offset to the shoulder,
        upper arm and forearm pitched in the sagittal plane
        (``elbow_angle = shoulder_pitch + elbow_pitch``), the whole arm then
        rolled about the base x axis by shoulder_roll.
        """
        body = np.asarray(body, dtype=np.float64).reshape(BODY_DIM)
        result: List[Tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        for side_sign, base in ((1.0, _LEFT_ARM_BASE), (-1.0, _RIGHT_ARM_BASE)):
            q_pitch = float(body[base + _IDX_SHOULDER_PITCH])
            q_roll = float(body[base + _IDX_SHOULDER_ROLL])
            q_elbow = float(body[base + _IDX_ELBOW_PITCH])
            shoulder = np.array(
                [
                    0.0,
                    side_sign * self.shoulder_offset_y_m,
                    self.shoulder_offset_z_m,
                ]
            )
            rot = _rot_x(q_roll)
            upper = rot @ np.array(
                [
                    self.upper_arm_m * math.sin(q_pitch),
                    0.0,
                    -self.upper_arm_m * math.cos(q_pitch),
                ]
            )
            fore = rot @ np.array(
                [
                    self.forearm_m * math.sin(q_pitch + q_elbow),
                    0.0,
                    -self.forearm_m * math.cos(q_pitch + q_elbow),
                ]
            )
            elbow = shoulder + upper
            wrist = elbow + fore
            result.append((shoulder, elbow, wrist))
        return result


def _rot_x(angle: float) -> np.ndarray:
    """Rotation matrix about the x axis."""
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
