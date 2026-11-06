"""Tests for the flat "open hand" view (no ROS, no hardware)."""
from __future__ import annotations

import numpy as np

from g1_viz.hand_view_core import (
    HandPainter,
    camera_panel,
    flatten_hand,
    link_poses_at_zero,
    side_by_side,
    stack_view,
)
from g1_viz.tactile_core import build_plate, merge_layout

URDF = """<robot name="t">
  <link name="root"/><link name="a"/><link name="b"/><link name="c"/>
  <joint name="j1" type="fixed"><parent link="root"/><child link="a"/><origin xyz="1 0 0" rpy="0 0 0"/></joint>
  <joint name="j2" type="revolute"><parent link="a"/><child link="b"/><origin xyz="0 2 0" rpy="0 0 1.5707963"/>
    <axis xyz="0 0 1"/></joint>
  <joint name="j3" type="fixed"><parent link="b"/><child link="c"/><origin xyz="1 0 0"/></joint>
</robot>"""


def test_link_poses_chain_the_joint_origins_at_zero_angle() -> None:
    poses = link_poses_at_zero(URDF)
    assert np.allclose(poses["a"][:3, 3], [1, 0, 0])
    assert np.allclose(poses["b"][:3, 3], [1, 2, 0])
    # b is yawed 90 degrees, so c (1 m along b's x) ends up 1 m along the root's y
    assert np.allclose(poses["c"][:3, 3], [1, 3, 0], atol=1e-6)


def _plate(region: str, link: str, rows: int, cols: int, lo, hi):
    return build_plate(region, link, rows, cols, lo, hi, merge_layout(""))


def _mini_hand():
    """A made-up open hand: fingers along +y, palm normal +z, thumb out along -x with plates standing on edge."""
    flat = ((-0.01, -0.012, -0.001), (0.01, 0.012, 0.001))  # finger plates: thin along z
    edge = ((-0.008, -0.001, -0.006), (0.008, 0.001, 0.006))  # thumb plates: thin along y (normal = up)
    plates = {
        "palm": _plate("palm", "palm", 8, 14, (-0.025, -0.015, -0.001), (0.025, 0.015, 0.001)),
        "middle_tip": _plate("middle_tip", "mt", 3, 3, *flat),
        "little_tip": _plate("little_tip", "lt", 3, 3, *flat),
        "index_tip": _plate("index_tip", "it", 3, 3, *flat),
        "thumb_pad": _plate("thumb_pad", "t1", 12, 8, *edge),
        "thumb_middle": _plate("thumb_middle", "t2", 3, 3, *edge),
        "thumb_nail": _plate("thumb_nail", "t3", 12, 8, *edge),
        "thumb_tip": _plate("thumb_tip", "t4", 3, 3, *edge),
    }

    def at(x: float, y: float, z: float = 0.0) -> np.ndarray:
        pose = np.eye(4)
        pose[:3, 3] = (x, y, z)
        return pose

    poses = {
        "palm": at(0, 0), "mt": at(0.0, 0.12), "lt": at(0.05, 0.10), "it": at(-0.025, 0.11),
        "t1": at(-0.05, -0.01, -0.01), "t2": at(-0.08, 0.0, -0.02), "t3": at(-0.10, 0.01, -0.025), "t4": at(-0.115, 0.015, -0.03),
    }
    return plates, poses


def _centre(flat, name):
    return flat[name].corners.reshape(-1, 2).mean(axis=0)


def test_thumb_is_on_the_viewers_left_for_the_right_hand_and_right_for_the_left() -> None:
    plates, poses = _mini_hand()
    right = flatten_hand("right", plates, poses)
    left = flatten_hand("left", plates, poses)
    assert _centre(right, "thumb_nail")[0] < _centre(right, "palm")[0]
    assert _centre(left, "thumb_nail")[0] > _centre(left, "palm")[0]
    # little finger on the side opposite to the index finger, fingertips ahead of the palm
    assert _centre(right, "little_tip")[0] > _centre(right, "index_tip")[0]
    assert _centre(left, "little_tip")[0] < _centre(left, "index_tip")[0]
    assert _centre(right, "middle_tip")[1] > _centre(right, "palm")[1]


def test_thumb_plates_are_unfolded_not_squashed_into_slivers() -> None:
    plates, poses = _mini_hand()
    flat = flatten_hand("right", plates, poses)

    def cell_area(region):
        corners = flat[region].corners[0, 0]
        edge_1, edge_2 = corners[1] - corners[0], corners[3] - corners[0]
        return abs(edge_1[0] * edge_2[1] - edge_1[1] * edge_2[0])

    # a 12 x 8 plate of 16 x 12 mm has cells of 1.33 x 1.5 mm = 2.0e-6 m^2; edge-on projection would give ~0
    assert cell_area("thumb_pad") > 1.5e-6
    assert cell_area("thumb_nail") > 1.5e-6
    # the thumb runs away from the palm: pad, middle, nail, tip in order of distance from the anchor
    anchor = _centre(flat, "thumb_pad")
    distances = [np.linalg.norm(_centre(flat, n) - anchor) for n in ("thumb_pad", "thumb_middle", "thumb_nail", "thumb_tip")]
    assert distances == sorted(distances)


def test_painter_colours_only_the_cells_above_the_threshold() -> None:
    plates, poses = _mini_hand()
    flat = flatten_hand("right", plates, poses)
    painter = HandPainter("right", flat, width=560, height=540, threshold=15.0, full_scale=400.0)
    grids = {name: np.zeros((p.rows, p.cols)) for name, p in plates.items()}
    quiet = painter.draw(grids)
    assert quiet.shape == (540, 560, 3) and quiet.dtype == np.uint8
    grids["palm"][0, 0] = 400.0
    busy = painter.draw(grids)
    centre = painter._px["palm"][0, 0].reshape(-1, 2).mean(axis=0).astype(int)
    assert tuple(busy[centre[1], centre[0]]) != tuple(quiet[centre[1], centre[0]])
    assert busy[centre[1], centre[0], 0] > 200 and busy[centre[1], centre[0], 2] < 80  # full scale = red
    assert (busy != quiet).any() and not np.array_equal(busy, painter.draw(None))


def test_camera_panel_and_stacking() -> None:
    frame = np.full((240, 640, 3), 200, dtype=np.uint8)  # wider than the panel: bars above and below
    panel = camera_panel(frame, 560, 420)
    assert panel.shape == (420, 560, 3)
    assert panel[210, 280, 0] == 200 and panel[0, 0, 0] != 200  # picture in the middle, bars at the top
    missing = camera_panel(None, 560, 420, "no image")
    assert missing.shape == (420, 560, 3) and missing.any()
    hand = np.zeros((540, 560, 3), dtype=np.uint8)
    assert stack_view(hand, panel, "below").shape == (960, 560, 3)
    assert stack_view(hand, None, "below").shape == (540, 560, 3)
    assert stack_view(hand, panel, "none").shape == (540, 560, 3)
    both = side_by_side([hand, hand])
    assert both.shape[0] == 540 and both.shape[1] > 1120
