"""rclpy-free 2D "open hand" view of the RH56E2-T1 tactile cells, with the palm camera next to it.

The 3D view (``tactile_core``) puts every region on its force-sensor link of the URDF. Here the same
cells are flattened: the URDF is walked at zero joint angles (= open hand, all fingers extended, thumb
out), every cell centre goes to the hand frame and is projected onto the palm plane, seen from the
palm side (the side with the sensors; the thumb is on the viewer's left for the right hand). Because
the positions come from the same plates as the 3D view, both views always agree on the orientation of
every grid (``tactile_core.DEFAULT_LAYOUT`` and its overrides).

Only numpy, OpenCV and the standard library are used.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from g1_viz.tactile_core import Plate, heat_color

_THUMB = ("thumb_pad", "thumb_middle", "thumb_nail", "thumb_tip")
_CELL_OFFSETS = ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))  # (row, col) signs of the 4 corners
_BG = (24, 24, 28)
_CELL_OFF = (52, 52, 60)
_CELL_LINE = (86, 86, 98)
_TEXT = (225, 225, 230)
_DIM = (150, 150, 160)


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis rpy: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def link_poses_at_zero(urdf_xml: str) -> Dict[str, np.ndarray]:
    """4x4 pose of every link in the URDF root frame with all joints at zero (origins only)."""
    root = ET.fromstring(urdf_xml)
    parent_of: Dict[str, Tuple[str, np.ndarray]] = {}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
        transform = np.eye(4)
        transform[:3, :3] = _rpy_matrix(*rpy)
        transform[:3, 3] = xyz
        parent_of[joint.find("child").get("link")] = (joint.find("parent").get("link"), transform)
    poses: Dict[str, np.ndarray] = {}

    def pose(link: str) -> np.ndarray:
        if link not in poses:
            if link in parent_of:
                parent, transform = parent_of[link]
                poses[link] = pose(parent) @ transform
            else:
                poses[link] = np.eye(4)
        return poses[link]

    for link in [e.get("name") for e in root.findall("link")]:
        pose(link)
    return poses


@dataclass(frozen=True)
class FlatRegion:
    """One region in the flat view: ``corners[r, c]`` are the 4 corners of cell (r, c), in metres.

    x grows to the viewer's right, y towards the fingertips.
    """

    name: str
    rows: int
    cols: int
    corners: np.ndarray  # (rows, cols, 4, 2)


def flatten_hand(side: str, plates: Dict[str, Plate], poses: Dict[str, np.ndarray]) -> Dict[str, FlatRegion]:
    """Project all plates onto the palm plane of an open hand, seen from the palm side."""
    world: Dict[str, np.ndarray] = {}
    for name, plate in plates.items():
        pose = poses[plate.link]
        world[name] = plate.centers @ pose[:3, :3].T + pose[:3, 3]  # (rows, cols, 3)

    def mean(name: str) -> np.ndarray:
        return world[name].reshape(-1, 3).mean(axis=0)

    origin = mean("palm")
    up = mean("middle_tip") - origin
    up /= np.linalg.norm(up)
    across = mean("little_tip") - mean("index_tip")
    across -= up * float(across @ up)
    across /= np.linalg.norm(across)
    # Seen from the palm side the thumb is on the viewer's left for the right hand, on the right for the left.
    wanted = -1.0 if side == "right" else 1.0
    if np.sign(float((mean("thumb_nail") - origin) @ across)) != wanted:
        across = -across

    xy_of: Dict[str, np.ndarray] = {
        name: np.stack([(w - origin) @ across, (w - origin) @ up], axis=-1) for name, w in world.items()
    }
    if all(name in world for name in _THUMB):
        xy_of.update(_unfold_thumb(world, origin, across, up))

    flat: Dict[str, FlatRegion] = {}
    for name, plate in plates.items():
        xy = xy_of[name]  # (rows, cols, 2)
        d_row = xy[1, 0] - xy[0, 0] if plate.rows > 1 else np.zeros(2)
        d_col = xy[0, 1] - xy[0, 0] if plate.cols > 1 else np.zeros(2)
        corners = np.stack(
            [xy + 0.5 * (sr * d_row + sc * d_col) for sr, sc in _CELL_OFFSETS], axis=2
        )  # (rows, cols, 4, 2)
        flat[name] = FlatRegion(name, plate.rows, plate.cols, corners)
    return flat


def _plate_normal(points: np.ndarray) -> np.ndarray:
    """Unit normal of a (nearly) planar grid of points (smallest principal direction)."""
    flat = points.reshape(-1, 3)
    return np.linalg.svd(flat - flat.mean(axis=0), full_matrices=False)[2][2]


def _unfold_thumb(
    world: Dict[str, np.ndarray], origin: np.ndarray, across: np.ndarray, up: np.ndarray
) -> Dict[str, np.ndarray]:
    """Lay the thumb plates flat along the thumb axis (open hand: they stand on edge to the palm plane).

    The plates all face the fingertips (normal ~ ``up``); the thumb is drawn as seen from that side,
    anchored where the thumb starts and pointing the way the thumb points in the palm plane.
    """
    normals = []
    for name in _THUMB:
        n = _plate_normal(world[name])
        normals.append(n if float(n @ up) >= 0.0 else -n)
    n = np.sum(normals, axis=0)
    n /= np.linalg.norm(n)
    p0 = world["thumb_pad"].reshape(-1, 3).mean(axis=0)
    axis = world["thumb_tip"].reshape(-1, 3).mean(axis=0) - p0
    axis /= np.linalg.norm(axis)
    along = axis - n * float(axis @ n)
    along /= np.linalg.norm(along)
    athwart = np.cross(along, n)  # (athwart, along, n) is right-handed with n towards the viewer
    pointing = np.array([float(axis @ across), float(axis @ up)])
    pointing /= np.linalg.norm(pointing)
    e_along = pointing
    e_athwart = np.array([pointing[1], -pointing[0]])
    anchor = np.array([float((p0 - origin) @ across), float((p0 - origin) @ up)])
    out: Dict[str, np.ndarray] = {}
    for name in _THUMB:
        rel = world[name] - p0
        out[name] = anchor + (rel @ athwart)[..., None] * e_athwart + (rel @ along)[..., None] * e_along
    return out


class HandPainter:
    """Draws one flat hand into an RGB image; the static part (cell outlines) is rendered once."""

    def __init__(
        self,
        side: str,
        flat: Dict[str, FlatRegion],
        width: int = 560,
        height: int = 540,
        threshold: float = 15.0,
        full_scale: float = 400.0,
    ) -> None:
        self.side, self.width, self.height = side, int(width), int(height)
        self._thr, self._full = float(threshold), float(full_scale)
        self._flat = flat
        top, bottom, margin = 58, 54, 18
        every = np.concatenate([r.corners.reshape(-1, 2) for r in flat.values()])
        lo, hi = every.min(axis=0), every.max(axis=0)
        scale = min((self.width - 2 * margin) / (hi[0] - lo[0]), (self.height - top - bottom) / (hi[1] - lo[1]))
        self._scale = scale
        ox = (self.width - scale * (hi[0] - lo[0])) / 2.0 - scale * lo[0]
        oy = top + (self.height - top - bottom - scale * (hi[1] - lo[1])) / 2.0 + scale * hi[1]
        self._px: Dict[str, np.ndarray] = {
            name: np.round(np.stack([ox + scale * r.corners[..., 0], oy - scale * r.corners[..., 1]], axis=-1)).astype(
                np.int32
            )
            for name, r in flat.items()
        }  # (rows, cols, 4, 2) pixels
        self._hull = {n: cv2.convexHull(p.reshape(-1, 2)) for n, p in self._px.items()}
        self._background = self._render_background()

    @staticmethod
    def _label(image: np.ndarray, text: str, cx: int, y: int) -> None:
        size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0][0]
        cv2.putText(image, text, (cx - size // 2, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, _DIM, 1, cv2.LINE_AA)

    def _render_background(self) -> np.ndarray:
        image = np.full((self.height, self.width, 3), _BG, dtype=np.uint8)
        for px in self._px.values():
            for r in range(px.shape[0]):
                for c in range(px.shape[1]):
                    cv2.fillConvexPoly(image, px[r, c], _CELL_OFF)
                    cv2.polylines(image, [px[r, c]], True, _CELL_LINE, 1, cv2.LINE_AA)
        for finger in ("little", "ring", "middle", "index", "thumb"):
            if f"{finger}_tip" in self._hull:
                x, y, w, _h = cv2.boundingRect(self._hull[f"{finger}_tip"])
                self._label(image, finger, x + w // 2, y - 7)
        if "palm" in self._hull:
            x, y, w, h = cv2.boundingRect(self._hull["palm"])
            self._label(image, "palm", x + w // 2, y + h + 15)
        label = f"{self.side.upper()} HAND  -  palm side"
        cv2.putText(image, label, (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, _TEXT, 1, cv2.LINE_AA)
        # colour bar: 0 .. full scale
        x0, x1, y0, y1 = 18, self.width - 18, self.height - 34, self.height - 20
        for x in range(x0, x1):
            r, g, b = heat_color((x - x0) / (x1 - x0) * self._full, self._full)
            cv2.line(image, (x, y0), (x, y1), (int(r * 255), int(g * 255), int(b * 255)), 1)
        cv2.putText(image, f"{self._thr:g}", (x0, y0 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, _DIM, 1, cv2.LINE_AA)
        end = f"{self._full:g} counts"
        size = cv2.getTextSize(end, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0][0]
        cv2.putText(image, end, (x1 - size, y0 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, _DIM, 1, cv2.LINE_AA)
        return image

    def draw(self, grids: Optional[Dict[str, np.ndarray]], note: str = "") -> np.ndarray:
        """RGB image. ``grids``: region -> (rows, cols) counts above the baseline, or None while waiting."""
        image = self._background.copy()
        peak_value, peak_region = 0.0, ""
        if grids is None:
            text = note or "waiting for the first tactile frames (keep the hand untouched)"
            cv2.putText(image, text, (14, self.height - 62), cv2.FONT_HERSHEY_SIMPLEX, 0.45, _DIM, 1, cv2.LINE_AA)
            return image
        for name, grid in grids.items():
            px = self._px[name]
            hot = np.argwhere(grid > self._thr)
            for r, c in hot:
                red, green, blue = heat_color(float(grid[r, c]), self._full)
                cv2.fillConvexPoly(image, px[r, c], (int(red * 255), int(green * 255), int(blue * 255)))
            if len(hot):
                cv2.polylines(image, [self._hull[name]], True, (255, 255, 255), 2, cv2.LINE_AA)
                region_peak = float(grid.max())
                if region_peak > peak_value:
                    peak_value, peak_region = region_peak, name
        if peak_region:
            text = f"peak {peak_value:.0f}  {peak_region}"
        else:
            text = note or "no contact"
        size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]
        cv2.putText(image, text, (self.width - size - 14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _TEXT, 1, cv2.LINE_AA)
        return image


def camera_panel(frame_rgb: Optional[np.ndarray], width: int, height: int, text: str = "") -> np.ndarray:
    """``frame_rgb`` scaled to fit width x height (aspect kept) on a dark panel; a message when there is none."""
    panel = np.full((height, width, 3), _BG, dtype=np.uint8)
    if frame_rgb is None:
        size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)[0]
        cv2.putText(panel, text, ((width - size[0]) // 2, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.55, _DIM, 1, cv2.LINE_AA)
        return panel
    h, w = frame_rgb.shape[:2]
    k = min(width / w, height / h)
    resized = cv2.resize(frame_rgb, (max(int(w * k), 1), max(int(h * k), 1)), interpolation=cv2.INTER_AREA)
    y0, x0 = (height - resized.shape[0]) // 2, (width - resized.shape[1]) // 2
    panel[y0 : y0 + resized.shape[0], x0 : x0 + resized.shape[1]] = resized
    return panel


def stack_view(hand: np.ndarray, camera: Optional[np.ndarray], position: str) -> np.ndarray:
    """Hand diagram and camera panel one above the other (``position``: below, above, none)."""
    if camera is None or position == "none":
        return hand
    return np.vstack([camera, hand]) if position == "above" else np.vstack([hand, camera])


def side_by_side(images: Sequence[np.ndarray], gap: int = 6) -> np.ndarray:
    """Images of equal height next to each other with a thin gap."""
    if len(images) == 1:
        return images[0]
    parts: List[np.ndarray] = []
    for index, image in enumerate(images):
        if index:
            parts.append(np.full((image.shape[0], gap, 3), 90, dtype=np.uint8))
        parts.append(image)
    return np.hstack(parts)
