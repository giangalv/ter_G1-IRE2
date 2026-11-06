"""Window with both hands laid out flat (palm side), the touched cells coloured, and the palm cameras.

Subscribes ``/g1/hand/{left,right}/tactile_raw`` (1062 cells, region order of
``g1_action.inspire_protocol.E2_TACTILE_REGIONS``) and, optionally,
``/g1/hand/{left,right}/camera/image_raw/compressed`` (palm camera JPEG, see
``tools/hand_cameras_ros2.sh`` and ``g1_unitree_bridge/hand_camera_relay`` in the repos). Draws everything
with OpenCV in one window: it is a plain window and not an RViz image, because the composed picture
(~1 MB) would not survive the laptop's 416 kB UDP receive buffer.

Keys: ``z`` takes the zero again (hands untouched!), ``q`` / Esc closes the window and the node.
Like the 3D view, the first ``baseline_frames`` frames are the zero; values are counts, not forces.
"""
from __future__ import annotations

import os
import time
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from g1_action.inspire_protocol import (
    E2_TACTILE_NUM_CELLS,
    E2_TACTILE_REGIONS,
    E2_TACTILE_SLICES,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt16MultiArray
from std_srvs.srv import Empty

from g1_viz.hand_view_core import (
    HandPainter,
    camera_panel,
    flatten_hand,
    link_poses_at_zero,
    side_by_side,
    stack_view,
)
from g1_viz.tactile_core import Baseline, build_plate, merge_layout, read_stl_bounds, region_link

_WINDOW = "G1 hands - palm side (z: re-zero, q: quit)"
_URDF = "g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf"


class Hand2D(Node):
    """Tactile cells + palm camera of both hands in one OpenCV window."""

    def __init__(self) -> None:
        super().__init__("hand_2d")
        self.declare_parameter("sides", ["left", "right"])
        self.declare_parameter("urdf", "")  # default: g1_description FTP model
        self.declare_parameter("meshes_dir", "")  # default: g1_description/meshes
        self.declare_parameter("layout_json", "")  # overrides of tactile_core.DEFAULT_LAYOUT
        self.declare_parameter("threshold", 15.0)  # counts above the baseline
        self.declare_parameter("full_scale", 400.0)  # counts drawn fully red
        self.declare_parameter("baseline_frames", 30)
        self.declare_parameter("camera", "below")  # below | above | none
        self.declare_parameter("camera_timeout_s", 2.0)
        self.declare_parameter("width", 560)
        self.declare_parameter("height", 540)
        self.declare_parameter("camera_height", 420)
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("show", True)
        self.declare_parameter("snapshot_path", "")  # write the picture here once a second (tests, documentation)

        gp = self.get_parameter
        self._sides = [str(s) for s in gp("sides").value]
        self._camera_mode = str(gp("camera").value)
        self._camera_timeout = float(gp("camera_timeout_s").value)
        self._width = int(gp("width").value)
        self._camera_height = int(gp("camera_height").value)
        self._period = 1.0 / max(float(gp("rate_hz").value), 1.0)
        self._show = bool(gp("show").value)
        self._snapshot = str(gp("snapshot_path").value)
        self._thr = float(gp("threshold").value)

        share = get_package_share_directory("g1_description")
        urdf_path = str(gp("urdf").value) or os.path.join(share, "urdf", _URDF)
        meshes_dir = str(gp("meshes_dir").value) or os.path.join(share, "meshes")
        with open(urdf_path, "r", encoding="utf-8") as handle:
            poses = link_poses_at_zero(handle.read())
        layout = merge_layout(str(gp("layout_json").value))
        frames = int(gp("baseline_frames").value)

        self._painter: Dict[str, HandPainter] = {}
        self._baseline: Dict[str, Baseline] = {}
        self._grids: Dict[str, Optional[Dict[str, np.ndarray]]] = {}
        self._camera: Dict[str, Optional[np.ndarray]] = {}
        self._camera_time: Dict[str, float] = {}
        self._last_raw: Dict[str, float] = {}
        for side in self._sides:
            plates = {}
            for region in E2_TACTILE_REGIONS:
                link = region_link(side, region.name)
                lo, hi = read_stl_bounds(os.path.join(meshes_dir, f"{link}.STL"))
                plates[region.name] = build_plate(region.name, link, region.rows, region.cols, lo, hi, layout, side=side)
            self._painter[side] = HandPainter(
                side,
                flatten_hand(side, plates, poses),
                width=self._width,
                height=int(gp("height").value),
                threshold=self._thr,
                full_scale=float(gp("full_scale").value),
            )
            self._baseline[side] = Baseline(E2_TACTILE_NUM_CELLS, frames)
            self._grids[side] = None
            self._camera[side] = None
            self._camera_time[side] = 0.0
            self._last_raw[side] = 0.0
            self.create_subscription(
                UInt16MultiArray,
                f"/g1/hand/{side}/tactile_raw",
                lambda msg, s=side: self._on_raw(s, msg),
                qos_profile_sensor_data,
            )
            if self._camera_mode != "none":
                self.create_subscription(
                    CompressedImage,
                    f"/g1/hand/{side}/camera/image_raw/compressed",
                    lambda msg, s=side: self._on_camera(s, msg),
                    qos_profile_sensor_data,
                )
        self.create_service(Empty, "/g1/viz/hand_2d/rezero", self._on_rezero)
        self._next_draw = 0.0
        self._shown_at = 0.0
        self._next_snapshot = 0.0
        self.get_logger().info(
            f"hand_2d up for {self._sides}: camera '{self._camera_mode}', threshold {self._thr:g} counts; "
            f"keep the hands untouched for {frames} frames (baseline)"
        )

    # ------------------------------------------------------------------ inputs
    def _on_rezero(self, _request, response):
        self.rezero()
        return response

    def rezero(self) -> None:
        for side in self._sides:
            self._baseline[side].reset()
            self._grids[side] = None
        self.get_logger().info("tactile baseline cleared, collecting a new one (do not touch)")

    def _on_raw(self, side: str, msg: UInt16MultiArray) -> None:
        if len(msg.data) != E2_TACTILE_NUM_CELLS:
            self.get_logger().warning(
                f"{side}: expected {E2_TACTILE_NUM_CELLS} cells, got {len(msg.data)}", throttle_duration_sec=5.0
            )
            return
        self._last_raw[side] = time.monotonic()
        baseline = self._baseline[side]
        if not baseline.ready:
            baseline.update(msg.data)
            return
        values = baseline.subtract(msg.data)
        self._grids[side] = {
            region.name: values[slice(*E2_TACTILE_SLICES[region.name])].reshape(region.rows, region.cols)
            for region in E2_TACTILE_REGIONS
        }

    def _on_camera(self, side: str, msg: CompressedImage) -> None:
        frame = cv2.imdecode(np.asarray(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning(f"{side}: palm camera JPEG could not be decoded", throttle_duration_sec=5.0)
            return
        self._camera[side] = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._camera_time[side] = time.monotonic()

    # ----------------------------------------------------------------- drawing
    def compose(self) -> np.ndarray:
        """The whole picture (RGB): both hands side by side, each with its camera panel."""
        now = time.monotonic()
        columns: List[np.ndarray] = []
        for side in self._sides:
            note = ""
            if self._last_raw[side] == 0.0 or now - self._last_raw[side] > 2.0:
                note = "no tactile data (hand bridge not running or hand off)"
                grids = None
            else:
                grids = self._grids[side]
            hand = self._painter[side].draw(grids, note)
            camera = None
            if self._camera_mode != "none":
                fresh = now - self._camera_time[side] <= self._camera_timeout
                camera = camera_panel(
                    self._camera[side] if fresh else None,
                    self._width,
                    self._camera_height,
                    "no palm camera image (tools/hand_cameras_ros2.sh start)",
                )
            columns.append(stack_view(hand, camera, self._camera_mode))
        return side_by_side(columns)

    def tick(self) -> bool:
        """Draw if due; returns False when the user closed the window."""
        now = time.monotonic()
        if now < self._next_draw:
            return True
        self._next_draw = now + self._period
        image = self.compose()
        if self._snapshot and now >= self._next_snapshot:
            self._next_snapshot = now + 1.0
            cv2.imwrite(self._snapshot, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        if not self._show:
            return True
        try:
            cv2.imshow(_WINDOW, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as error:
            self.get_logger().error(f"cannot open a window ({error}); continuing without one (show:=false)")
            self._show = False
            return True
        if key in (ord("q"), 27):
            return False
        if key == ord("z"):
            self.rezero()
        if self._shown_at == 0.0:
            self._shown_at = now
        elif now - self._shown_at > 3.0 and cv2.getWindowProperty(_WINDOW, cv2.WND_PROP_AUTOSIZE) < 0:
            return False  # window closed with its X button
        return True


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = Hand2D()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if not node.tick():
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
