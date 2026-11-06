"""RViz markers of the RH56E2-T1 tactile cells, drawn on the force-sensor links of the URDF.

Subscribes ``/g1/hand/{left,right}/tactile_raw`` (1062 cells, region order of
``g1_action.inspire_protocol.E2_TACTILE_REGIONS``) and publishes
``/g1/viz/tactile/{left,right}`` (MarkerArray): per region a CUBE_LIST with one cube per cell above
the threshold, coloured blue -> red with the pressure, plus a translucent sphere on the plate whose
size and colour follow the strongest cell, so a touch is visible from across the room.

The first ``baseline_frames`` frames are taken as the zero of every cell (keep the hands untouched
when the node starts); call ``/g1/viz/tactile/rezero`` to take it again. Counts are NOT forces.
"""
from __future__ import annotations

import os
from typing import Dict, List, Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from g1_action.inspire_protocol import (
    E2_TACTILE_NUM_CELLS,
    E2_TACTILE_REGIONS,
    E2_TACTILE_SLICES,
)
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import ColorRGBA, UInt16MultiArray
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

from g1_viz.tactile_core import (
    Baseline,
    Plate,
    active_cells,
    build_plate,
    heat_color,
    merge_layout,
    read_stl_bounds,
    region_link,
)


class TactileMarkers(Node):
    """Turn raw tactile frames into coloured cells on the hand model."""

    def __init__(self) -> None:
        super().__init__("tactile_markers")
        self.declare_parameter("sides", ["left", "right"])
        self.declare_parameter("threshold", 15.0)  # counts above the baseline
        self.declare_parameter("full_scale", 400.0)  # counts drawn fully red
        self.declare_parameter("baseline_frames", 30)
        self.declare_parameter("meshes_dir", "")  # default: g1_description/meshes
        self.declare_parameter("layout_json", "")  # overrides of tactile_core.DEFAULT_LAYOUT
        self.declare_parameter("lifetime_s", 0.5)

        self._thr = float(self.get_parameter("threshold").value)
        self._full = float(self.get_parameter("full_scale").value)
        self._lifetime = Duration(
            sec=0, nanosec=int(float(self.get_parameter("lifetime_s").value) * 1e9)
        )
        meshes_dir = str(self.get_parameter("meshes_dir").value) or os.path.join(
            get_package_share_directory("g1_description"), "meshes"
        )
        layout = merge_layout(str(self.get_parameter("layout_json").value))
        frames = int(self.get_parameter("baseline_frames").value)

        self._plates: Dict[str, List[Plate]] = {}
        self._baseline: Dict[str, Baseline] = {}
        self._pubs = {}
        sides = [str(s) for s in self.get_parameter("sides").value]
        for side in sides:
            plates = []
            for region in E2_TACTILE_REGIONS:
                link = region_link(side, region.name)
                lo, hi = read_stl_bounds(os.path.join(meshes_dir, f"{link}.STL"))
                plates.append(
                    build_plate(region.name, link, region.rows, region.cols, lo, hi, layout, side=side)
                )
            self._plates[side] = plates
            self._baseline[side] = Baseline(E2_TACTILE_NUM_CELLS, frames)
            self._pubs[side] = self.create_publisher(MarkerArray, f"/g1/viz/tactile/{side}", 5)
            self.create_subscription(
                UInt16MultiArray,
                f"/g1/hand/{side}/tactile_raw",
                lambda msg, s=side: self._on_raw(s, msg),
                qos_profile_sensor_data,
            )
        self.create_service(Empty, "/g1/viz/tactile/rezero", self._on_rezero)
        self.get_logger().info(
            f"tactile_markers up for {sides}: {len(E2_TACTILE_REGIONS)} regions/hand, threshold "
            f"{self._thr:g} counts, full scale {self._full:g}; keep the hands untouched for "
            f"{frames} frames (baseline)"
        )

    def _on_rezero(self, _request, response):
        for baseline in self._baseline.values():
            baseline.reset()
        self.get_logger().info("tactile baseline cleared, collecting a new one (do not touch)")
        return response

    def _on_raw(self, side: str, msg: UInt16MultiArray) -> None:
        if len(msg.data) != E2_TACTILE_NUM_CELLS:
            self.get_logger().warning(
                f"{side}: expected {E2_TACTILE_NUM_CELLS} cells, got {len(msg.data)}",
                throttle_duration_sec=5.0,
            )
            return
        baseline = self._baseline[side]
        if not baseline.ready:
            baseline.update(msg.data)
            if baseline.ready:
                self.get_logger().info(f"{side}: tactile baseline taken")
            return
        values = baseline.subtract(msg.data)

        out = MarkerArray()
        for idx, plate in enumerate(self._plates[side]):
            lo, hi = E2_TACTILE_SLICES[plate.region]
            grid = values[lo:hi].reshape(plate.rows, plate.cols)
            out.markers.append(self._cells_marker(idx, plate, grid))
            out.markers.append(self._halo_marker(idx, plate, grid))
        self._pubs[side].publish(out)

    def _base_marker(self, plate: Plate, ns: str, idx: int, kind: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = plate.link  # stamp 0 = latest TF
        marker.ns = ns
        marker.id = idx
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = self._lifetime
        return marker

    def _cells_marker(self, idx: int, plate: Plate, grid: np.ndarray) -> Marker:
        marker = self._base_marker(plate, "cells", idx, Marker.CUBE_LIST)
        marker.scale.x, marker.scale.y, marker.scale.z = (float(v) for v in plate.cell_size)
        for r, c in active_cells(grid, self._thr):
            x, y, z = plate.centers[r, c]
            marker.points.append(Point(x=float(x), y=float(y), z=float(z)))
            red, green, blue = heat_color(float(grid[r, c]), self._full)
            marker.colors.append(ColorRGBA(r=red, g=green, b=blue, a=1.0))
        return marker

    def _halo_marker(self, idx: int, plate: Plate, grid: np.ndarray) -> Marker:
        marker = self._base_marker(plate, "halo", idx, Marker.SPHERE)
        peak = float(grid.max())
        if peak <= self._thr:
            marker.scale.x = marker.scale.y = marker.scale.z = 1e-4
            marker.color = ColorRGBA(a=0.0)
            return marker
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            float(v) for v in plate.center
        )
        diameter = 0.014 + 0.026 * min(peak / self._full, 1.0)
        marker.scale.x = marker.scale.y = marker.scale.z = diameter
        red, green, blue = heat_color(peak, self._full)
        marker.color = ColorRGBA(r=red, g=green, b=blue, a=0.45)
        return marker


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = TactileMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
