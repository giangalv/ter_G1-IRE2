"""Tests for the rclpy-free depth maths."""
from __future__ import annotations

import struct

import cv2
import numpy as np
import pytest

from g1_unitree_bridge.depth_core import decode_compressed_depth, depth_to_metres, depth_to_points, intrinsics


def test_metres_and_invalid_pixels() -> None:
    raw = np.array([[0, 1000], [2500, 65535]], dtype=np.uint16)
    m = depth_to_metres(raw)
    assert m.dtype == np.float32
    assert np.isnan(m[0, 0]) and np.isclose(m[0, 1], 1.0) and np.isclose(m[1, 0], 2.5)


def test_back_projection_of_the_principal_point_and_an_offset() -> None:
    m = np.full((4, 4), 2.0, dtype=np.float32)
    pts = depth_to_points(m, (100.0, 100.0, 2.0, 2.0), stride=1)
    assert pts.shape == (16, 3) and pts.dtype == np.float32
    centre = pts[(np.isclose(pts[:, 0], 0.0)) & (np.isclose(pts[:, 1], 0.0))]
    assert len(centre) == 1 and np.isclose(centre[0, 2], 2.0)
    # pixel u=3 is one pixel right of cx=2: x = 1 * 2 / 100
    assert np.isclose(pts[:, 0].max(), 0.02) and np.isclose(pts[:, 1].max(), 0.02)


def test_stride_and_range_filter() -> None:
    m = np.full((6, 6), 1.0, dtype=np.float32)
    m[0, 0] = np.nan
    m[2, 2] = 10.0  # beyond max_range
    m[4, 4] = 0.05  # closer than min_range
    pts = depth_to_points(m, (50.0, 50.0, 3.0, 3.0), stride=2, max_range=6.0)
    assert len(pts) == 9 - 3  # 3x3 sampled pixels minus nan, far, near


def test_intrinsics_validation() -> None:
    assert intrinsics((300.0, 0, 212.0, 0, 301.0, 120.0, 0, 0, 1)) == (300.0, 301.0, 212.0, 120.0)
    with pytest.raises(ValueError):
        intrinsics((1.0,) * 4)
    with pytest.raises(ValueError):
        depth_to_points(np.ones((2, 2), np.float32), (0.0, 0.0, 0.0, 0.0))


def _compressed_depth(img: np.ndarray) -> bytes:
    """What tools/realsense_encoder_node.py on the Jetson sends: image_transport header + 16-bit PNG."""
    ok, png = cv2.imencode(".png", img, [int(cv2.IMWRITE_PNG_COMPRESSION), 1])
    assert ok
    return struct.pack("<iff", 0, 0.0, 0.0) + png.tobytes()


def test_compressed_depth_round_trip_is_lossless() -> None:
    rng = np.random.default_rng(1)
    raw = rng.integers(0, 6000, size=(240, 424), dtype=np.uint16)
    raw[:30] = 0  # holes
    out = decode_compressed_depth(_compressed_depth(raw))
    assert out is not None and out.dtype == np.uint16 and out.shape == raw.shape
    assert np.array_equal(out, raw)


def test_compressed_depth_rejects_what_is_not_a_16_bit_png() -> None:
    assert decode_compressed_depth(b"") is None
    assert decode_compressed_depth(b"\x00" * 12) is None  # header only
    assert decode_compressed_depth(b"\x00" * 12 + b"not a png") is None
    ok, png = cv2.imencode(".png", np.zeros((4, 4), np.uint8))
    assert ok and decode_compressed_depth(b"\x00" * 12 + png.tobytes()) is None  # 8 bit
    ok, png = cv2.imencode(".png", np.zeros((4, 4, 3), np.uint8))
    assert ok and decode_compressed_depth(b"\x00" * 12 + png.tobytes()) is None  # colour
