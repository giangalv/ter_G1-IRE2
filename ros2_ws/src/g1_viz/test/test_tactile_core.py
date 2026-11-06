"""Tests for the rclpy-free tactile geometry and colouring."""
from __future__ import annotations

import struct

import numpy as np
import pytest

from g1_viz.tactile_core import (
    Baseline,
    active_cells,
    build_plate,
    default_axes,
    heat_color,
    merge_layout,
    read_stl_bounds,
    region_link,
)


def test_region_links_follow_the_finger_order() -> None:
    assert region_link("left", "index_pad") == "left_index_force_sensor_1"
    assert region_link("left", "index_nail") == "left_index_force_sensor_2"
    assert region_link("right", "little_tip") == "right_little_force_sensor_3"
    assert region_link("right", "thumb_pad") == "right_thumb_force_sensor_1"
    assert region_link("right", "thumb_middle") == "right_thumb_force_sensor_2"
    assert region_link("left", "thumb_nail") == "left_thumb_force_sensor_3"
    assert region_link("left", "thumb_tip") == "left_thumb_force_sensor_4"
    assert region_link("left", "palm") == "left_palm_force_sensor"


def test_unknown_region_is_rejected() -> None:
    with pytest.raises(ValueError):
        region_link("left", "index_knuckle")


def test_default_axes() -> None:
    assert default_axes(12, 8) == ("long", "short")
    assert default_axes(8, 14) == ("short", "long")


LO, HI = (-0.0057, -0.0073, -0.0017), (0.0057, 0.0073, 0.0017)  # an 11.4 x 14.6 x 3.4 mm plate


def test_grid_stays_inside_the_plate_and_covers_it() -> None:
    plate = build_plate("index_pad", "l", 10, 8, LO, HI, {}, thickness_margin=0.0)
    assert plate.centers.shape == (10, 8, 3)
    assert (np.abs(plate.centers[:, :, 0]) < HI[0]).all() and (np.abs(plate.centers[:, :, 1]) < HI[1]).all()
    # 10 rows run along the long (y) side, 8 columns along the short (x) side
    assert plate.centers[0, 0, 1] < plate.centers[-1, 0, 1]
    assert plate.centers[0, 0, 0] < plate.centers[0, -1, 0]
    assert np.isclose(plate.cell_size[1] * 10, HI[1] - LO[1])
    assert np.isclose(plate.cell_size[0] * 8, HI[0] - LO[0])


def test_flip_reverses_the_direction() -> None:
    plain = build_plate("r", "l", 10, 8, LO, HI, {})
    flipped = build_plate("r", "l", 10, 8, LO, HI, {"r": {"flip_rows": True, "flip_cols": True}})
    assert np.allclose(plain.centers[0, 0], flipped.centers[-1, -1])


def test_palm_default_columns_run_along_the_short_side_towards_minus() -> None:
    lo, hi = (-0.025, -0.015, -0.001), (0.025, 0.015, 0.001)  # 50 x 30 mm
    layout = merge_layout()
    palm = build_plate("palm", "l", 8, 14, lo, hi, layout)
    assert palm.centers[0, 0, 1] > palm.centers[0, -1, 1]  # column 0 at +y (finger side)
    assert palm.centers[0, 0, 0] < palm.centers[-1, 0, 0]  # the 8 rows run along x (long side)
    assert np.isclose(palm.centers[:, 0, 0].ptp(), 0.050 * 7 / 8)  # 8 rows across 50 mm
    assert np.isclose(palm.centers[0, :, 1].ptp(), 0.030 * 13 / 14)  # 14 columns along 30 mm


def test_layout_override_json() -> None:
    layout = merge_layout('{"index_pad": {"flip_rows": true}, "palm": {"flip_cols": false}}')
    assert layout["index_pad"]["flip_rows"] is True
    assert layout["palm"]["flip_cols"] is False
    assert layout["palm"]["cols_axis"] == "short"  # untouched keys survive


def test_same_axis_for_rows_and_cols_is_rejected() -> None:
    with pytest.raises(ValueError):
        build_plate("x", "l", 3, 3, LO, HI, {"x": {"rows_axis": "long", "cols_axis": "long"}})


def test_baseline_collects_then_freezes() -> None:
    base = Baseline(3, frames=2)
    assert not base.ready
    base.update([10, 20, 30])
    base.update([20, 20, 30])
    assert base.ready and np.allclose(base.value, [15, 20, 30])
    base.update([999, 999, 999])  # ignored once ready
    assert np.allclose(base.value, [15, 20, 30])
    assert np.allclose(base.subtract([10, 50, 30]), [0, 30, 0])  # never negative
    base.reset()
    assert not base.ready


def test_heat_color_is_blue_to_red_and_clips() -> None:
    low, high = heat_color(0.0, 100.0), heat_color(100.0, 100.0)
    assert low[2] > low[0] and high[0] > high[2]
    assert heat_color(1e9, 100.0) == high and heat_color(-5.0, 100.0) == low


def test_active_cells() -> None:
    grid = np.array([[0.0, 20.0], [5.0, 30.0]])
    assert active_cells(grid, 15.0) == [(0, 1), (1, 1)]


def test_stl_bounds_roundtrip(tmp_path) -> None:
    tris = [((0, 0, 1), ((0, 0, 0), (1, 0, 0), (0, 2, 0.5)))]
    blob = bytearray(80) + struct.pack("<I", len(tris))
    for n, verts in tris:
        blob += struct.pack("<3f", *n) + b"".join(struct.pack("<3f", *v) for v in verts) + b"\0\0"
    path = tmp_path / "t.STL"
    path.write_bytes(bytes(blob))
    lo, hi = read_stl_bounds(str(path))
    assert np.allclose(lo, [0, 0, 0]) and np.allclose(hi, [1, 2, 0.5])
    path.write_bytes(b"solid x\n" + b"0" * 200)
    with pytest.raises(ValueError):
        read_stl_bounds(str(path))


def test_side_specific_layout_wins_and_only_applies_to_that_hand() -> None:
    layout = merge_layout()
    right = build_plate("thumb_pad", "r", 12, 8, LO, HI, layout, side="right")
    left = build_plate("thumb_pad", "l", 12, 8, LO, HI, layout, side="left")
    plain = build_plate("thumb_pad", "x", 12, 8, LO, HI, {})
    assert np.allclose(left.centers, plain.centers)  # left thumb untouched
    assert np.allclose(right.centers[0, 0], plain.centers[-1, 0])  # rows reversed on the right
    assert np.allclose(right.centers[:, :, 0], plain.centers[:, :, 0])  # columns not
