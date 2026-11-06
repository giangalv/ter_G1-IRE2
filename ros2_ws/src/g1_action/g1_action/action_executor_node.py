"""``action_executor_node``: 50 Hz realtime executor for the G1 U6 VLA stack.

Thin ROS 2 shell around the rclpy-free cores in :mod:`g1_action.executor_core`,
:mod:`g1_action.safety` and :mod:`g1_action.blending`.

Responsibilities (CONVENTIONS.md):

* ONLY writer of ``/g1/arm_sdk``, ``/g1/cmd_vel`` and ``/g1/hand/*/cmd`` on
  the VLA path.
* Receding-horizon execution of ``/g1/vla/action_chunk`` with replan-overlap
  blending and impedance HOLD during replan gaps / after ``chunk_invalid``.
* Dual-path FAST <-> PRECISE blending over ``blend_window_steps`` ticks.
* Five-stage safety pipeline: dual-encoder fault (latched ESTOP), velocity
  clamp, torque clamp, workspace block, self-collision capsule block.
* Hand execution with an optional 100 Hz force-position hybrid loop
  (``grasp_precision`` subtasks) including TACTILE_SLIP reaction.
* ``navigate_to`` subtasks: P-controller toward the DynaMem object pose.
* On ESTOP: arm_sdk weight ramps to 0 over 0.5 s, cmd_vel zeroed, hands hold.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

from g1_interfaces.msg import (
    ActionChunk,
    ArmCmd,
    HandCmd,
    ObjectPose,
    SubtaskToken,
    TactileArray,
)

from g1_action.blending import DualPathBlender
from g1_action.executor_core import (
    ACTION_DIM,
    ARM_DIM,
    BODY_DIM,
    FORCE_DIM,
    LEFT_FORCE_SLICE,
    LEFT_HAND_SLICE,
    MODE_FAST,
    MODE_PRECISE,
    RIGHT_FORCE_SLICE,
    RIGHT_HAND_SLICE,
    ChunkBuffer,
    DecodedChunk,
    HandForceController,
    NavigationController,
    normalize_hand_positions,
)
from g1_action.qos import COMMAND_QOS, LATCHED_QOS, SENSOR_QOS
from g1_action.safety import SafetyPipeline

_SIDES = ("left", "right")
_HAND_SLICES = {"left": LEFT_HAND_SLICE, "right": RIGHT_HAND_SLICE}
_FORCE_SLICES = {"left": LEFT_FORCE_SLICE, "right": RIGHT_FORCE_SLICE}


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rad) of a quaternion (ZYX convention)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class ActionExecutorNode(Node):
    """Safety-critical realtime executor (regular Node, realtime CPU)."""

    def __init__(self) -> None:
        super().__init__("action_executor_node")

        # -- parameters (spec defaults; overridden by g1_bringup params) ----
        self.declare_parameter("execution_rate_hz", 50.0)
        self.declare_parameter("blend_window_steps", 25)
        self.declare_parameter("max_joint_velocity_rad_s", 2.5)
        self.declare_parameter("max_joint_torque_nm", 120.0)
        self.declare_parameter("dual_encoder_fault_threshold", 0.05)
        self.declare_parameter("replan_overlap", 0.5)
        self.declare_parameter(
            "workspace_bounds", [-0.2, 0.8, -0.6, 0.6, 0.0, 1.6]
        )
        self.declare_parameter("arm_sdk_weight", 1.0)
        self.declare_parameter("stiffness_nm_per_rad", 80.0)
        self.declare_parameter("upper_arm_m", 0.28)
        self.declare_parameter("forearm_m", 0.26)
        self.declare_parameter("shoulder_offset_y_m", 0.15)
        self.declare_parameter("shoulder_offset_z_m", 1.1)
        self.declare_parameter("torso_capsule_z_min_m", 0.7)
        self.declare_parameter("torso_capsule_radius_m", 0.07)
        self.declare_parameter("forearm_capsule_radius_m", 0.035)
        self.declare_parameter("self_collision_margin_m", 0.01)
        self.declare_parameter("hand_force_kp", 0.02)
        self.declare_parameter("hand_force_ki", 0.001)
        self.declare_parameter("hand_rate_hz", 100.0)
        self.declare_parameter(
            "hand_joint_min_rad", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.declare_parameter(
            "hand_joint_max_rad", [1.7, 1.7, 1.7, 1.7, 0.6, 1.3]
        )
        self.declare_parameter("nav_kp_linear", 0.8)
        self.declare_parameter("nav_kp_angular", 1.5)
        self.declare_parameter("nav_max_linear_m_s", 0.4)
        self.declare_parameter("nav_max_angular_rad_s", 0.5)
        self.declare_parameter("nav_goal_tolerance_m", 0.15)
        self.declare_parameter("estop_release_s", 0.5)
        self.declare_parameter("encoder_delta_rate_hz", 10.0)

        gp = self.get_parameter
        self._rate_hz = float(gp("execution_rate_hz").value)
        self._arm_sdk_weight = float(gp("arm_sdk_weight").value)
        self._estop_release_s = float(gp("estop_release_s").value)
        self._hand_q_min = np.asarray(
            gp("hand_joint_min_rad").value, dtype=np.float64
        ).reshape(6)
        self._hand_q_max = np.asarray(
            gp("hand_joint_max_rad").value, dtype=np.float64
        ).reshape(6)
        hand_rate_hz = float(gp("hand_rate_hz").value)

        # -- rclpy-free cores -------------------------------------------------
        self._buffer = ChunkBuffer(
            replan_overlap=float(gp("replan_overlap").value)
        )
        self._blender = DualPathBlender(int(gp("blend_window_steps").value))
        self._pipeline = SafetyPipeline(
            rate_hz=self._rate_hz,
            max_joint_velocity_rad_s=float(gp("max_joint_velocity_rad_s").value),
            max_joint_torque_nm=float(gp("max_joint_torque_nm").value),
            dual_encoder_fault_threshold=float(
                gp("dual_encoder_fault_threshold").value
            ),
            workspace_bounds=list(gp("workspace_bounds").value),
            stiffness_nm_per_rad=float(gp("stiffness_nm_per_rad").value),
            upper_arm_m=float(gp("upper_arm_m").value),
            forearm_m=float(gp("forearm_m").value),
            shoulder_offset_y_m=float(gp("shoulder_offset_y_m").value),
            shoulder_offset_z_m=float(gp("shoulder_offset_z_m").value),
            torso_capsule_z_min_m=float(gp("torso_capsule_z_min_m").value),
            torso_capsule_radius_m=float(gp("torso_capsule_radius_m").value),
            forearm_capsule_radius_m=float(gp("forearm_capsule_radius_m").value),
            self_collision_margin_m=float(gp("self_collision_margin_m").value),
        )
        self._nav = NavigationController(
            kp_linear=float(gp("nav_kp_linear").value),
            kp_angular=float(gp("nav_kp_angular").value),
            max_linear_m_s=float(gp("nav_max_linear_m_s").value),
            max_angular_rad_s=float(gp("nav_max_angular_rad_s").value),
            goal_tolerance_m=float(gp("nav_goal_tolerance_m").value),
        )
        self._hand_controllers: Dict[str, HandForceController] = {
            side: HandForceController(
                kp=float(gp("hand_force_kp").value),
                ki=float(gp("hand_force_ki").value),
                rate_hz=hand_rate_hz,
            )
            for side in _SIDES
        }

        # -- state ---------------------------------------------------------
        self._primary: Optional[np.ndarray] = None
        self._secondary: Optional[np.ndarray] = None
        self._prev_cmd: Optional[np.ndarray] = None
        self._last_mode_action: Dict[int, np.ndarray] = {}
        self._tactile: Dict[str, Optional[np.ndarray]] = {s: None for s in _SIDES}
        self._hand_targets: Dict[str, Optional[Tuple[np.ndarray, np.ndarray, bool]]] = {
            s: None for s in _SIDES
        }
        self._last_hand_cmd: Dict[str, Optional[Tuple[np.ndarray, np.ndarray]]] = {
            s: None for s in _SIDES
        }
        self._slip_logged: Dict[str, bool] = {s: False for s in _SIDES}
        self._subtask_text = ""
        self._odom_pose: Optional[Tuple[float, float, float]] = None
        self._nav_target: Optional[Tuple[float, float]] = None
        self._estop_active = False
        self._estop_source = ""
        self._estop_tick = 0
        self._encoder_fault_sent = False

        # -- publishers ------------------------------------------------------
        self._arm_pub = self.create_publisher(ArmCmd, "/g1/arm_sdk", COMMAND_QOS)
        self._twist_pub = self.create_publisher(Twist, "/g1/cmd_vel", COMMAND_QOS)
        self._hand_pubs = {
            side: self.create_publisher(
                HandCmd, f"/g1/hand/{side}/cmd", COMMAND_QOS
            )
            for side in _SIDES
        }
        self._encoder_fault_pub = self.create_publisher(
            Bool, "/g1/diagnostics/encoder_fault", LATCHED_QOS
        )
        self._encoder_delta_pub = self.create_publisher(
            Float64MultiArray, "/g1/diagnostics/encoder_delta", COMMAND_QOS
        )
        self._estop_pub = self.create_publisher(Bool, "/g1/estop", LATCHED_QOS)

        # -- subscriptions -----------------------------------------------------
        self.create_subscription(
            ActionChunk, "/g1/vla/action_chunk", self._on_action_chunk, COMMAND_QOS
        )
        self.create_subscription(
            JointState, "/g1/joint_states", self._on_joint_states, SENSOR_QOS
        )
        self.create_subscription(
            JointState,
            "/g1/joint_states_secondary",
            self._on_joint_states_secondary,
            SENSOR_QOS,
        )
        self.create_subscription(
            Bool,
            "/g1/memory/dynamem/chunk_invalid",
            self._on_chunk_invalid,
            LATCHED_QOS,
        )
        for side in _SIDES:
            self.create_subscription(
                TactileArray,
                f"/g1/hand/{side}/tactile",
                lambda msg, s=side: self._on_tactile(s, msg),
                SENSOR_QOS,
            )
        self.create_subscription(
            SubtaskToken, "/g1/planner/subtask", self._on_subtask, COMMAND_QOS
        )
        self.create_subscription(Odometry, "/g1/odom", self._on_odom, SENSOR_QOS)
        self.create_subscription(
            ObjectPose,
            "/g1/memory/dynamem/object_pose",
            self._on_object_pose,
            COMMAND_QOS,
        )
        self.create_subscription(Bool, "/g1/estop", self._on_estop, LATCHED_QOS)

        # -- timers ------------------------------------------------------------
        self._tick_timer = self.create_timer(1.0 / self._rate_hz, self._on_tick)
        self._hand_timer = self.create_timer(
            1.0 / hand_rate_hz, self._on_hand_tick
        )
        self._delta_timer = self.create_timer(
            1.0 / float(gp("encoder_delta_rate_hz").value),
            self._on_encoder_delta_tick,
        )

        self.get_logger().info(
            f"action_executor_node up: {self._rate_hz:.0f} Hz, "
            f"blend window {self._blender.window} ticks, "
            f"replan overlap {self._buffer.replan_overlap:.2f}"
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _on_action_chunk(self, msg: ActionChunk) -> None:
        """Decode and buffer a new chunk (ignored while ESTOP is active)."""
        if self._estop_active:
            return
        try:
            chunk = DecodedChunk.from_flat(
                msg.chunk_length,
                msg.positions,
                msg.velocities,
                msg.hand_force,
                mode=int(msg.mode),
                timestamp=float(msg.timestamp),
                action_entropy=float(msg.action_entropy),
            )
        except ValueError as exc:
            self.get_logger().error(f"Rejected malformed ActionChunk: {exc}")
            return
        self._buffer.push(chunk)

    def _on_joint_states(self, msg: JointState) -> None:
        self._primary = self._body_vector(msg)

    def _on_joint_states_secondary(self, msg: JointState) -> None:
        self._secondary = self._body_vector(msg)

    @staticmethod
    def _body_vector(msg: JointState) -> np.ndarray:
        """BODY-41 vector from a JointState (fixed order per CONVENTIONS.md)."""
        body = np.zeros(BODY_DIM, dtype=np.float64)
        n = min(len(msg.position), BODY_DIM)
        body[:n] = np.asarray(msg.position[:n], dtype=np.float64)
        return body

    def _on_chunk_invalid(self, msg: Bool) -> None:
        if msg.data:
            self._buffer.invalidate()
            self.get_logger().warning(
                "DynaMem chunk_invalid: discarding chunks, entering HOLD "
                "(body holds last targets, grip force retained)"
            )

    def _on_tactile(self, side: str, msg: TactileArray) -> None:
        if len(msg.pressure) == 15:
            self._tactile[side] = np.asarray(msg.pressure, dtype=np.float64)

    def _on_subtask(self, msg: SubtaskToken) -> None:
        text = msg.text.strip()
        if text != self._subtask_text:
            self._subtask_text = text
            # New subtask: re-arm the tactile hybrid loop and slip latches.
            for side in _SIDES:
                self._hand_controllers[side].reset()
                self._slip_logged[side] = False
            self.get_logger().info(f"Subtask: '{text}'")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._odom_pose = (p.x, p.y, _yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _on_object_pose(self, msg: ObjectPose) -> None:
        p = msg.pose.pose.position
        self._nav_target = (p.x, p.y)

    def _on_estop(self, msg: Bool) -> None:
        """External estop stops output; False releases only external stops."""
        if msg.data:
            if not self._estop_active:
                self._engage_estop("external /g1/estop", publish_estop=False)
                self._estop_source = "external"
        else:
            if (
                self._estop_active
                and self._estop_source == "external"
                and not self._pipeline.estop_latched
            ):
                self._estop_active = False
                self._estop_tick = 0
                self.get_logger().warning("External ESTOP released; resuming")

    # ------------------------------------------------------------------
    # 50 Hz main tick
    # ------------------------------------------------------------------

    def _on_tick(self) -> None:
        if self._estop_active:
            self._tick_estop()
            return
        if self._primary is None:
            # No proprioception yet: command nothing but keep the base still.
            self._publish_twist(0.0, 0.0)
            return

        if self._prev_cmd is None:
            self._prev_cmd = self._fallback_state()

        out = self._buffer.step(self._fallback_state())

        # Dual-path blending on the concatenated (positions | hand_force).
        self._blender.notify_mode(out.mode)
        combined = np.concatenate([out.positions, out.hand_force])
        other_mode = MODE_PRECISE if out.mode == MODE_FAST else MODE_FAST
        blended = self._blender.step(
            combined, self._last_mode_action.get(other_mode)
        )
        self._last_mode_action[out.mode] = combined
        target = blended[:ACTION_DIM]
        hand_force = blended[ACTION_DIM:]

        decision = self._pipeline.check(
            target, self._prev_cmd, self._primary, self._secondary
        )
        if decision.estop:
            self._publish_encoder_fault()
            self._engage_estop(decision.reason, publish_estop=True)
            self._estop_source = "internal"
            self._tick_estop()
            return
        if not decision.ok:
            self.get_logger().warning(
                f"Safety block: {decision.reason}", throttle_duration_sec=1.0
            )

        self._prev_cmd = decision.action.copy()

        arm = ArmCmd()
        arm.header.stamp = self.get_clock().now().to_msg()
        arm.q = [float(v) for v in decision.action[:ARM_DIM]]
        if decision.ok and not out.holding:
            arm.dq = [float(v) for v in out.velocities[:ARM_DIM]]
        else:
            arm.dq = [0.0] * ARM_DIM
        arm.tau_ff = [0.0] * ARM_DIM
        arm.weight = self._arm_sdk_weight
        self._arm_pub.publish(arm)

        self._update_hand_commands(decision.action, hand_force)
        self._tick_navigation()

    def _fallback_state(self) -> np.ndarray:
        """(53,) hold target: measured body + last commanded hand joints."""
        body = (
            self._primary.copy()
            if self._primary is not None
            else np.zeros(BODY_DIM, dtype=np.float64)
        )
        hand = (
            self._prev_cmd[BODY_DIM:].copy()
            if self._prev_cmd is not None
            else np.zeros(ACTION_DIM - BODY_DIM, dtype=np.float64)
        )
        return np.concatenate([body, hand])

    # ------------------------------------------------------------------
    # Hand execution
    # ------------------------------------------------------------------

    def _update_hand_commands(
        self, action: np.ndarray, hand_force: np.ndarray
    ) -> None:
        """Publish 50 Hz HandCmds; hybrid targets go to the 100 Hz loop."""
        use_tactile = self._subtask_text.startswith("grasp_precision")
        for side in _SIDES:
            q_norm = normalize_hand_positions(
                action[_HAND_SLICES[side]], self._hand_q_min, self._hand_q_max
            )
            forces = np.maximum(hand_force[_FORCE_SLICES[side]], 0.0)
            self._hand_targets[side] = (q_norm, forces, use_tactile)
            if use_tactile:
                continue  # 100 Hz hybrid loop owns publication.
            self._hand_controllers[side].reset()
            self._slip_logged[side] = False
            self._publish_hand_cmd(side, q_norm, forces, use_tactile=False)

    def _on_hand_tick(self) -> None:
        """100 Hz force-position hybrid loop (grasp_precision subtasks)."""
        if self._estop_active:
            return
        for side in _SIDES:
            target = self._hand_targets[side]
            if target is None or not target[2]:
                continue
            q_norm, forces, _ = target
            pressures = self._tactile[side]
            if pressures is None:
                # No tactile stream yet: fall back to pure position control.
                self._publish_hand_cmd(side, q_norm, forces, use_tactile=True)
                continue
            result = self._hand_controllers[side].update(q_norm, forces, pressures)
            if result.slip_detected and not self._slip_logged[side]:
                self._slip_logged[side] = True
                self.get_logger().warning(
                    f"TACTILE_SLIP on {side} hand: mean pressure "
                    f"{result.mean_pressure:.2f} N; freezing position, "
                    f"bumping force limit 20 %"
                )
            self._publish_hand_cmd(
                side, result.position, result.force_limit, use_tactile=True
            )

    def _publish_hand_cmd(
        self,
        side: str,
        position: np.ndarray,
        force_limit: np.ndarray,
        use_tactile: bool,
    ) -> None:
        msg = HandCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(v) for v in np.asarray(position).reshape(6)]
        msg.force_limit = [float(v) for v in np.asarray(force_limit).reshape(5)]
        msg.use_tactile_feedback = bool(use_tactile)
        self._hand_pubs[side].publish(msg)
        self._last_hand_cmd[side] = (
            np.asarray(position, dtype=np.float64).reshape(6).copy(),
            np.asarray(force_limit, dtype=np.float64).reshape(5).copy(),
        )

    # ------------------------------------------------------------------
    # Navigation subtasks
    # ------------------------------------------------------------------

    def _tick_navigation(self) -> None:
        """P-control toward the DynaMem target while in ``navigate_to``."""
        if (
            self._subtask_text.startswith("navigate_to")
            and self._odom_pose is not None
            and self._nav_target is not None
        ):
            x, y, yaw = self._odom_pose
            tx, ty = self._nav_target
            linear, angular = self._nav.compute(x, y, yaw, tx, ty)
            self._publish_twist(linear, angular)
        else:
            self._publish_twist(0.0, 0.0)

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._twist_pub.publish(msg)

    # ------------------------------------------------------------------
    # ESTOP handling
    # ------------------------------------------------------------------

    def _engage_estop(self, reason: str, publish_estop: bool) -> None:
        """Latch the node-level ESTOP and start the smooth arm release."""
        if self._estop_active:
            return
        self._estop_active = True
        self._estop_source = "internal" if publish_estop else "external"
        self._estop_tick = 0
        self.get_logger().fatal(f"ESTOP engaged: {reason}")
        if publish_estop:
            estop = Bool()
            estop.data = True
            self._estop_pub.publish(estop)
        # Hands hold position (do NOT release the grip).
        for side in _SIDES:
            last = self._last_hand_cmd[side]
            if last is not None:
                self._publish_hand_cmd(side, last[0], last[1], use_tactile=False)
        self._publish_twist(0.0, 0.0)

    def _publish_encoder_fault(self) -> None:
        if not self._encoder_fault_sent:
            self._encoder_fault_sent = True
            fault = Bool()
            fault.data = True
            self._encoder_fault_pub.publish(fault)

    def _tick_estop(self) -> None:
        """Ramp arm_sdk weight to 0 over ``estop_release_s``; zero cmd_vel."""
        release_ticks = max(1.0, self._estop_release_s * self._rate_hz)
        scale = max(0.0, 1.0 - self._estop_tick / release_ticks)
        self._estop_tick += 1
        if scale > 0.0 or self._estop_tick <= release_ticks + 1:
            hold = (
                self._prev_cmd
                if self._prev_cmd is not None
                else self._fallback_state()
            )
            arm = ArmCmd()
            arm.header.stamp = self.get_clock().now().to_msg()
            arm.q = [float(v) for v in hold[:ARM_DIM]]
            arm.dq = [0.0] * ARM_DIM
            arm.tau_ff = [0.0] * ARM_DIM
            arm.weight = self._arm_sdk_weight * scale
            self._arm_pub.publish(arm)
        self._publish_twist(0.0, 0.0)

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def _on_encoder_delta_tick(self) -> None:
        """Publish |primary - secondary| per joint at 10 Hz."""
        if self._primary is None or self._secondary is None:
            return
        delta = SafetyPipeline.encoder_delta(self._primary, self._secondary)
        msg = Float64MultiArray()
        msg.data = [float(v) for v in delta]
        self._encoder_delta_pub.publish(msg)


def main(args: Optional[list] = None) -> None:
    """Entry point."""
    rclpy.init(args=args)
    node = ActionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
