"""rclpy-free geometry and colouring for the tactile view of the RH56E2-T1 hands.

The hand reports 1062 cells in 17 regions (``g1_action.inspire_protocol.E2_TACTILE_REGIONS``).
Unitree's FTP URDF has one flat force-sensor link per region; the cell grid of a region is laid
on the bounding box of that link's mesh. The mapping of region -> link follows the order along the
finger (checked against the mesh sizes: the two big plates are ``pad`` and ``nail``, the small one
is the ``tip``):

    finger   tip = force_sensor_3, nail = force_sensor_2, pad = force_sensor_1
    thumb    tip = force_sensor_4, nail = force_sensor_3, middle = force_sensor_2, pad = force_sensor_1
    palm     left_palm_force_sensor / right_palm_force_sensor

The orientation of the grid inside a plate (which cell index runs along which side) is NOT written
down by the manufacturer; ``DEFAULT_LAYOUT`` is a first guess and every entry can be overridden
(see ``merge_layout``) once checked by touching the hand.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

_FINGER_SENSOR = {"pad": 1, "nail": 2, "tip": 3}
_THUMB_SENSOR = {"pad": 1, "middle": 2, "nail": 3, "tip": 4}

# Grid orientation per region: ``rows_axis``/``cols_axis`` say which in-plane side of the plate the
# row / column index runs along ("long" or "short"), ``flip_rows``/``flip_cols`` reverse the
# direction. Anything not listed gets the rule in ``default_axes`` and no flip.
# Keys are a region name ("index_pad", both hands) or "<side>_<region>" ("right_thumb_pad", that hand
# only; it wins over the plain region entry).
DEFAULT_LAYOUT: Dict[str, Dict[str, object]] = {
    # Earlier touch tests: column 0 is at the base of the fingers, column 13 towards the wrist, and
    # the fingers lie along the +short axis of the plate in the URDF, so the columns run along the
    # short side and increase towards -short.
    "palm": {"rows_axis": "long", "cols_axis": "short", "flip_cols": True},
    # Touched on 2026-11-05: on the right hand the first phalanx of the thumb works the opposite
    # way. Read as rows reversed along the phalanx; the left thumb has not been tried yet.
    "right_thumb_pad": {"flip_rows": True},
}


def region_link(side: str, region: str) -> str:
    """URDF link of the force-sensor plate that carries ``region`` (e.g. ``index_pad``)."""
    if region == "palm":
        return f"{side}_palm_force_sensor"
    finger, _, part = region.partition("_")
    table = _THUMB_SENSOR if finger == "thumb" else _FINGER_SENSOR
    if part not in table:
        raise ValueError(f"unknown tactile region '{region}'")
    return f"{side}_{finger}_force_sensor_{table[part]}"


def read_stl_bounds(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """(min, max) corners of a BINARY STL file, in the mesh's own frame (metres for Unitree)."""
    with open(path, "rb") as handle:
        data = handle.read()
    n_tri = int(np.frombuffer(data[80:84], dtype="<u4")[0])
    if len(data) != 84 + 50 * n_tri:
        raise ValueError(f"{path}: not a binary STL")
    tri = np.frombuffer(
        data[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    )
    vertices = tri["v"].reshape(-1, 3).astype(np.float64)
    return vertices.min(axis=0), vertices.max(axis=0)


def merge_layout(overrides_json: str = "") -> Dict[str, Dict[str, object]]:
    """DEFAULT_LAYOUT with a JSON override string applied (``{"index_pad": {"flip_rows": true}}``)."""
    layout = {k: dict(v) for k, v in DEFAULT_LAYOUT.items()}
    if overrides_json.strip():
        for region, entry in json.loads(overrides_json).items():
            layout.setdefault(region, {}).update(entry)
    return layout


def default_axes(rows: int, cols: int) -> Tuple[str, str]:
    """More rows than columns: rows follow the long side of the plate (and vice versa)."""
    return ("long", "short") if rows >= cols else ("short", "long")


@dataclass(frozen=True)
class Plate:
    """Cell centres of one region, expressed in the frame of its force-sensor link."""

    region: str
    link: str
    rows: int
    cols: int
    centers: np.ndarray  # (rows, cols, 3) metres
    cell_size: np.ndarray  # (3,) metres: size of one cell along the link's x, y, z
    center: np.ndarray  # (3,) middle of the plate


def build_plate(
    region: str,
    link: str,
    rows: int,
    cols: int,
    lo: Sequence[float],
    hi: Sequence[float],
    layout: Dict[str, Dict[str, object]],
    thickness_margin: float = 0.002,
    side: str = "",
) -> Plate:
    """Lay a rows x cols grid on the bounding box [lo, hi] of the plate mesh."""
    lo_a, hi_a = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
    extent = hi_a - lo_a
    center = (lo_a + hi_a) / 2.0
    order = np.argsort(extent)  # thin, short, long
    thin, short, long_ = int(order[0]), int(order[1]), int(order[2])
    entry = {**layout.get(region, {}), **layout.get(f"{side}_{region}", {})}
    d_rows, d_cols = default_axes(rows, cols)
    side_axis = {"long": long_, "short": short}
    row_axis = side_axis[str(entry.get("rows_axis", d_rows))]
    col_axis = side_axis[str(entry.get("cols_axis", d_cols))]
    if row_axis == col_axis:
        raise ValueError(f"{region}: rows and columns cannot share an axis")
    row_sign = -1.0 if entry.get("flip_rows") else 1.0
    col_sign = -1.0 if entry.get("flip_cols") else 1.0

    cell = np.zeros(3)
    cell[row_axis] = extent[row_axis] / rows
    cell[col_axis] = extent[col_axis] / cols
    cell[thin] = extent[thin] + thickness_margin  # poke out of the (opaque) mesh on both sides

    centers = np.tile(center, (rows, cols, 1))
    row_off = (np.arange(rows) + 0.5 - rows / 2.0) * cell[row_axis] * row_sign
    col_off = (np.arange(cols) + 0.5 - cols / 2.0) * cell[col_axis] * col_sign
    centers[:, :, row_axis] += row_off[:, None]
    centers[:, :, col_axis] += col_off[None, :]
    return Plate(region, link, rows, cols, centers, cell, center)


class Baseline:
    """Per-cell offset: mean of the first ``frames`` frames (hand untouched), then frozen."""

    def __init__(self, n_cells: int, frames: int = 30) -> None:
        self._n = n_cells
        self._frames = max(int(frames), 1)
        self.reset()

    def reset(self) -> None:
        self._sum = np.zeros(self._n, dtype=np.float64)
        self._count = 0
        self.value = np.zeros(self._n, dtype=np.float64)

    @property
    def ready(self) -> bool:
        return self._count >= self._frames

    def update(self, raw: Sequence[float]) -> None:
        """Feed a frame while collecting; ignored once the baseline is complete."""
        if self.ready:
            return
        self._sum += np.asarray(raw, dtype=np.float64)
        self._count += 1
        if self.ready:
            self.value = self._sum / self._count

    def subtract(self, raw: Sequence[float]) -> np.ndarray:
        """Counts above the baseline, never negative."""
        return np.maximum(np.asarray(raw, dtype=np.float64) - self.value, 0.0)


# Blue -> cyan -> green -> yellow -> red.
_STOPS = np.array(
    [[0.10, 0.25, 1.00], [0.00, 0.85, 0.95], [0.10, 0.90, 0.20], [1.00, 0.90, 0.00], [1.00, 0.10, 0.05]]
)


def heat_color(value: float, full_scale: float) -> Tuple[float, float, float]:
    """RGB for ``value`` counts, saturating at ``full_scale``."""
    x = min(max(value / max(full_scale, 1e-9), 0.0), 1.0) * (len(_STOPS) - 1)
    i = min(int(x), len(_STOPS) - 2)
    t = x - i
    rgb = _STOPS[i] * (1.0 - t) + _STOPS[i + 1] * t
    return float(rgb[0]), float(rgb[1]), float(rgb[2])


def active_cells(values: np.ndarray, threshold: float) -> List[Tuple[int, int]]:
    """(row, col) of the cells above ``threshold``."""
    rr, cc = np.nonzero(values > threshold)
    return list(zip(rr.tolist(), cc.tolist()))
