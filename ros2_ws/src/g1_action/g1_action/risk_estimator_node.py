"""``risk_estimator_node``: 10 Hz aggregate risk scoring for dual-path dispatch.

Thin ROS 2 shell around :mod:`g1_action.risk_core`. Publishes
``g1_interfaces/RiskScore`` on ``/g1/risk/score`` at 10 Hz from:

* depth image + LiDAR minimum distance inside the arm workspace
  (``collision_proximity``),
* DynaMem object pose confidence (``grasp_uncertainty``),
* the latest ActionChunk entropy (``action_entropy``),
* tactile mean-pressure spike detection with ContactState support
  (``tactile_anomaly``),
* task phase from the planner subtask, plus the tactile-VLA overrides
  (``slip_detected`` forces precise; a confirmed stable grasp returns to
  the fast path).
"""

from __future__ import annotations

import math
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool

from g1_interfaces.msg import (
    ActionChunk,
    ContactState,
    ObjectPose,
    RiskScore,
    SubtaskToken,
    TactileArray,
)

from g1_action.qos import COMMAND_QOS, SENSOR_QOS
from g1_action.risk_core import (
    TactileAnomalyDetector,
    TaskPhase,
    collision_proximity,
    decide_trigger_precise,
    min_depth_distance,
    min_point_distance,
    phase_from_subtask,
)

_SIDES = ("left", "right")


def _depth_image_to_array(msg: Image) -> Optional[np.ndarray]:
    """Decode a 32FC1 depth image into (H, W) float32 meters, or None."""
    if msg.encoding != "32FC1" or msg.height == 0 or msg.width == 0:
        return None
    row_bytes = msg.width * 4
    if msg.step < row_bytes:
        return None
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    needed = msg.height * msg.step
    if buf.size < needed:
        return None
    rows = buf[:needed].reshape(msg.height, msg.step)[:, :row_bytes]
    dtype = ">f4" if msg.is_bigendian else "<f4"
    return rows.copy().view(dtype).reshape(msg.height, msg.width)


def _pointcloud_to_xyz(msg: PointCloud2, stride: int = 1) -> np.ndarray:
    """Extract (N, 3) float32 x/y/z columns from a PointCloud2."""
    offsets: Dict[str, int] = {f.name: f.offset for f in msg.fields}
    if not all(axis in offsets for axis in ("x", "y", "z")):
        return np.zeros((0, 3), dtype=np.float64)
    count = msg.width * msg.height
    if count == 0 or msg.point_step < 12:
        return np.zeros((0, 3), dtype=np.float64)
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    needed = count * msg.point_step
    if buf.size < needed:
        return np.zeros((0, 3), dtype=np.float64)
    records = buf[:needed].reshape(count, msg.point_step)[:: max(int(stride), 1)]
    dtype = ">f4" if msg.is_bigendian else "<f4"
    cols = []
    for axis in ("x", "y", "z"):
        off = offsets[axis]
        cols.append(records[:, off : off + 4].copy().view(dtype).reshape(-1))
    return np.stack(cols, axis=1).astype(np.float64)


class RiskEstimatorNode(Node):
    """Aggregate risk estimator (regular Node, realtime CPU, 10 Hz)."""

    def __init__(self) -> None:
        super().__init__("risk_estimator_node")

        # -- parameters ------------------------------------------------------
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("collision_d0_m", 0.35)
        self.declare_parameter("collision_scale_m", 0.1)
        self.declare_parameter("depth_subsample", 8)
        self.declare_parameter("depth_near_clip_m", 0.05)
        self.declare_parameter("depth_max_range_m", 2.0)
        self.declare_parameter("lidar_subsample", 4)
        self.declare_parameter(
            "workspace_bounds", [-0.2, 0.8, -0.6, 0.6, 0.0, 1.6]
        )
        self.declare_parameter("collision_threshold", 0.7)
        self.declare_parameter("uncertainty_threshold", 0.6)
        self.declare_parameter("entropy_threshold", 0.4)
        self.declare_parameter("tactile_threshold", 0.3)
        self.declare_parameter("tactile_window_s", 1.0)
        self.declare_parameter("tactile_z_threshold", 3.0)
        self.declare_parameter("tactile_noise_floor_n", 0.05)
        self.declare_parameter("contact_instability_floor", 0.5)
        self.declare_parameter("sensor_timeout_s", 1.0)
        self.declare_parameter("slip_hold_s", 1.0)

        gp = self.get_parameter
        self._rate_hz = float(gp("rate_hz").value)
        self._d0 = float(gp("collision_d0_m").value)
        self._scale = float(gp("collision_scale_m").value)
        self._depth_subsample = int(gp("depth_subsample").value)
        self._depth_near = float(gp("depth_near_clip_m").value)
        self._bounds = np.asarray(
            gp("workspace_bounds").value, dtype=np.float64
        ).reshape(6)
        # Workspace-box membership of a depth pixel is approximated by its
        # forward range (no extrinsics at this level).
        self._depth_far = min(
            float(gp("depth_max_range_m").value), float(self._bounds[1])
        )
        self._lidar_subsample = int(gp("lidar_subsample").value)
        self._th_collision = float(gp("collision_threshold").value)
        self._th_uncertainty = float(gp("uncertainty_threshold").value)
        self._th_entropy = float(gp("entropy_threshold").value)
        self._th_tactile = float(gp("tactile_threshold").value)
        self._contact_floor = float(gp("contact_instability_floor").value)
        self._sensor_timeout_s = float(gp("sensor_timeout_s").value)
        self._slip_hold_s = float(gp("slip_hold_s").value)

        window = max(2, int(round(self._rate_hz * float(gp("tactile_window_s").value))))
        self._detector = TactileAnomalyDetector(
            window_size=window,
            z_threshold=float(gp("tactile_z_threshold").value),
            min_std=float(gp("tactile_noise_floor_n").value),
        )

        # -- state ---------------------------------------------------------
        self._depth_min = math.inf
        self._depth_time = 0.0
        self._lidar_min = math.inf
        self._lidar_time = 0.0
        self._grasp_uncertainty = 0.0  # 0 until a first ObjectPose arrives
        self._action_entropy = 0.0
        self._hand_mean: Dict[str, float] = {}
        self._hand_time: Dict[str, float] = {}
        self._contact: Optional[ContactState] = None
        self._contact_time = 0.0
        self._phase = TaskPhase.UNKNOWN
        self._slip_time = -math.inf
        self._depth_warned = False

        # -- subscriptions -----------------------------------------------------
        self.create_subscription(
            Image, "/g1/camera/depth/image", self._on_depth, SENSOR_QOS
        )
        self.create_subscription(
            PointCloud2, "/g1/lidar/points", self._on_lidar, SENSOR_QOS
        )
        self.create_subscription(
            ObjectPose,
            "/g1/memory/dynamem/object_pose",
            self._on_object_pose,
            COMMAND_QOS,
        )
        self.create_subscription(
            ActionChunk, "/g1/vla/action_chunk", self._on_action_chunk, COMMAND_QOS
        )
        for side in _SIDES:
            self.create_subscription(
                TactileArray,
                f"/g1/hand/{side}/tactile",
                lambda msg, s=side: self._on_tactile(s, msg),
                SENSOR_QOS,
            )
        self.create_subscription(
            ContactState, "/g1/tactile/contact_state", self._on_contact, SENSOR_QOS
        )
        self.create_subscription(
            Bool, "/g1/tactile/slip_detected", self._on_slip, COMMAND_QOS
        )
        self.create_subscription(
            SubtaskToken, "/g1/planner/subtask", self._on_subtask, COMMAND_QOS
        )

        # -- publisher / timer ---------------------------------------------------
        self._score_pub = self.create_publisher(
            RiskScore, "/g1/risk/score", COMMAND_QOS
        )
        self.create_timer(1.0 / self._rate_hz, self._on_timer)

        self.get_logger().info(
            f"risk_estimator_node up: {self._rate_hz:.0f} Hz, "
            f"d0={self._d0} m, window={window} samples"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_depth(self, msg: Image) -> None:
        depth = _depth_image_to_array(msg)
        if depth is None:
            if not self._depth_warned:
                self._depth_warned = True
                self.get_logger().warning(
                    f"Unsupported depth image (encoding '{msg.encoding}'); "
                    "expected 32FC1"
                )
            return
        self._depth_min = min_depth_distance(
            depth,
            stride=self._depth_subsample,
            near_clip_m=self._depth_near,
            far_clip_m=self._depth_far,
        )
        self._depth_time = self._now_s()

    def _on_lidar(self, msg: PointCloud2) -> None:
        points = _pointcloud_to_xyz(msg, stride=self._lidar_subsample)
        self._lidar_min = min_point_distance(points, self._bounds)
        self._lidar_time = self._now_s()

    def _on_object_pose(self, msg: ObjectPose) -> None:
        self._grasp_uncertainty = float(
            min(max(1.0 - msg.confidence, 0.0), 1.0)
        )

    def _on_action_chunk(self, msg: ActionChunk) -> None:
        self._action_entropy = float(min(max(msg.action_entropy, 0.0), 1.0))

    def _on_tactile(self, side: str, msg: TactileArray) -> None:
        if len(msg.pressure) > 0:
            self._hand_mean[side] = float(
                np.asarray(msg.pressure, dtype=np.float64).mean()
            )
            self._hand_time[side] = self._now_s()

    def _on_contact(self, msg: ContactState) -> None:
        self._contact = msg
        self._contact_time = self._now_s()

    def _on_slip(self, msg: Bool) -> None:
        if msg.data:
            self._slip_time = self._now_s()

    def _on_subtask(self, msg: SubtaskToken) -> None:
        self._phase = phase_from_subtask(msg.text)

    # ------------------------------------------------------------------
    # 10 Hz scoring
    # ------------------------------------------------------------------

    def _on_timer(self) -> None:
        now = self._now_s()
        fresh = lambda t: (now - t) <= self._sensor_timeout_s  # noqa: E731

        # collision_proximity from min(depth, lidar) inside the workspace.
        min_dist = math.inf
        if fresh(self._depth_time):
            min_dist = min(min_dist, self._depth_min)
        if fresh(self._lidar_time):
            min_dist = min(min_dist, self._lidar_min)
        cp = collision_proximity(min_dist, d0=self._d0, scale=self._scale)

        # tactile_anomaly: spike detection on the mean pressure.
        means = [
            self._hand_mean[s]
            for s in _SIDES
            if s in self._hand_mean and fresh(self._hand_time.get(s, 0.0))
        ]
        anomaly = self._detector.update(float(np.mean(means))) if means else 0.0

        contact = self._contact if fresh(self._contact_time) else None
        grasp_stable = bool(contact.grasp_stable) if contact is not None else False
        if (
            contact is not None
            and any(contact.finger_contact)
            and not contact.grasp_stable
        ):
            # Contacted but unstable fingers: floor the anomaly signal.
            anomaly = max(anomaly, self._contact_floor)

        slip = (now - self._slip_time) <= self._slip_hold_s

        trigger = decide_trigger_precise(
            collision_proximity=cp,
            grasp_uncertainty=self._grasp_uncertainty,
            action_entropy=self._action_entropy,
            tactile_anomaly=anomaly,
            phase=self._phase,
            slip_detected=slip,
            grasp_stable=grasp_stable,
            collision_threshold=self._th_collision,
            uncertainty_threshold=self._th_uncertainty,
            entropy_threshold=self._th_entropy,
            tactile_threshold=self._th_tactile,
        )

        score = RiskScore()
        score.header.stamp = self.get_clock().now().to_msg()
        score.collision_proximity = float(cp)
        score.grasp_uncertainty = float(self._grasp_uncertainty)
        score.action_entropy = float(self._action_entropy)
        score.tactile_anomaly = float(anomaly)
        score.trigger_precise = bool(trigger)
        self._score_pub.publish(score)


def main(args: Optional[list] = None) -> None:
    """Entry point."""
    rclpy.init(args=args)
    node = RiskEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
