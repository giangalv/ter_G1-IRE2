"""rclpy-free depth maths: RealSense Z16 image -> metres and a decimated XYZ point cloud."""
from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np

# image_transport "compressedDepth" message data = 12-byte header (int32 format, float depthParam[2]) + a 16-bit PNG
DEPTH_HEADER_BYTES = 12


def depth_to_metres(raw: np.ndarray, scale: float = 0.001) -> np.ndarray:
    """Z16 counts (1 mm on a D435) -> float32 metres; 0 (no measurement) becomes NaN."""
    metres = raw.astype(np.float32) * np.float32(scale)
    metres[raw == 0] = np.nan
    return metres


def depth_to_points(
    metres: np.ndarray,
    k: Tuple[float, float, float, float],
    stride: int = 2,
    max_range: float = 6.0,
    min_range: float = 0.1,
) -> np.ndarray:
    """Pinhole back-projection into the optical frame (x right, y down, z forward).

    ``k`` is (fx, fy, cx, cy); every ``stride``-th pixel in both directions is used and
    NaN / out-of-range pixels are dropped. Returns an (N, 3) float32 array.
    """
    fx, fy, cx, cy = k
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera_info has no intrinsics (fx/fy <= 0)")
    z = metres[::stride, ::stride]
    v, u = np.mgrid[0 : metres.shape[0] : stride, 0 : metres.shape[1] : stride]
    ok = np.isfinite(z) & (z > min_range) & (z < max_range)
    z, u, v = z[ok], u[ok].astype(np.float32), v[ok].astype(np.float32)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float32)


def intrinsics(k_matrix: Optional[Tuple[float, ...]]) -> Tuple[float, float, float, float]:
    """(fx, fy, cx, cy) from a row-major 3x3 CameraInfo.k."""
    if k_matrix is None or len(k_matrix) != 9:
        raise ValueError("CameraInfo.k must have 9 values")
    return float(k_matrix[0]), float(k_matrix[4]), float(k_matrix[2]), float(k_matrix[5])


def decode_compressed_depth(data: bytes) -> Optional[np.ndarray]:
    """Data of a ``16UC1; compressedDepth png`` message (made on the Jetson by tools/realsense_encoder_node.py in the base
    repo) -> uint16 (H, W) array, or None if it is not a 16-bit single-channel PNG after the header."""
    if len(data) <= DEPTH_HEADER_BYTES:
        return None
    img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8)[DEPTH_HEADER_BYTES:], cv2.IMREAD_UNCHANGED)
    if img is None or img.ndim != 2 or img.dtype != np.uint16:
        return None
    return img
