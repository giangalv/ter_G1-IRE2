"""Palm-camera capture core for the G1 Inspire hands (rclpy-free).

Everything the ROS shell (:mod:`g1_hand_camera.hand_camera_node`) needs, kept
free of rclpy so it runs in plain pytest and on a machine without ROS:

* :class:`OpenCvFrameSource` -- V4L2 capture through OpenCV, one per hand;
* :class:`MockFrameSource` -- deterministic synthetic frames (``mock`` mode);
* :class:`ReconnectBackoff` / :class:`RateLimiter` -- hot-plug retry schedule
  and frame decimation;
* camera calibration loading (ROS ``camera_calibration`` YAML) and JPEG
  encoding.

Frames are always returned as contiguous ``(H, W, 3) uint8`` **RGB** arrays
(OpenCV delivers BGR), matching the ``rgb8`` convention of the workspace.

The module stays importable on Python 3.8 (the Jetson Orin NX system Python).
"""

from __future__ import annotations

import array
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

try:  # Heavy / host-specific dependency (see CONVENTIONS.md).
    import cv2  # type: ignore

    HAS_CV2 = True
except ImportError:  # pragma: no cover - exercised only without OpenCV
    cv2 = None  # type: ignore[assignment]
    HAS_CV2 = False

try:  # Only needed for calibration files.
    import yaml  # type: ignore

    HAS_YAML = True
except ImportError:  # pragma: no cover - exercised only without PyYAML
    yaml = None  # type: ignore[assignment]
    HAS_YAML = False

SIDES = ("left", "right")

#: Stable device names of the palm cameras on the lab robot. The two
#: "Abham Image" cameras share one USB serial number, so ``/dev/v4l/by-id`` is
#: useless; the mapping follows the Jetson hub PORT (verified with a
#: cover-the-lens test: port 2.2.1 = right hand, 2.2.2 = left hand). Swapping
#: the cables swaps the hands.
DEFAULT_DEVICES: Dict[str, str] = {
    "right": "/dev/v4l/by-path/platform-3610000.xhci-usb-0:2.2.1:1.0-video-index0",
    "left": "/dev/v4l/by-path/platform-3610000.xhci-usb-0:2.2.2:1.0-video-index0",
}

DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30.0


class CameraError(RuntimeError):
    """Capture failure (cannot open, read failed, bad frame, bad calibration)."""


# ---------------------------------------------------------------------------
# Frame helpers
# ---------------------------------------------------------------------------


def bgr_to_rgb8(frame: np.ndarray) -> np.ndarray:
    """OpenCV frame -> contiguous ``(H, W, 3) uint8`` RGB.

    Accepts BGR ``(H, W, 3)``, single-channel ``(H, W)`` or ``(H, W, 1)``
    frames (replicated to 3 channels).

    Raises:
        CameraError: for anything that is not an 8-bit image.
    """
    arr = np.asarray(frame)
    if arr.dtype != np.uint8:
        raise CameraError(f"expected an 8-bit frame, got {arr.dtype}")
    if arr.ndim == 2:
        return np.ascontiguousarray(np.repeat(arr[:, :, None], 3, axis=2))
    if arr.ndim == 3 and arr.shape[2] == 1:
        return np.ascontiguousarray(np.repeat(arr, 3, axis=2))
    if arr.ndim == 3 and arr.shape[2] == 3:
        return np.ascontiguousarray(arr[:, :, ::-1])
    raise CameraError(f"unsupported frame shape {arr.shape}")


def to_u8_array(data: bytes) -> "array.array":
    """``bytes`` -> ``array.array('B')`` for a ``uint8[]`` message field.

    rclpy (Humble) validates a ``bytes`` assigned to ``msg.data`` element by
    element: ~80 ms for a 640x480 rgb8 frame, i.e. it caps the stream at ~12
    fps. An ``array.array('B')`` takes the fast path (~0.1 ms).
    """
    out = array.array("B")
    out.frombytes(data)
    return out


def encode_jpeg(rgb: np.ndarray, quality: int = 85) -> bytes:
    """JPEG-encode an RGB frame.

    Raises:
        CameraError: when OpenCV is missing or the encoder fails.
    """
    if not HAS_CV2:
        raise CameraError("OpenCV (cv2) is required for JPEG encoding")
    q = int(min(max(quality, 1), 100))
    ok, buf = cv2.imencode(
        ".jpg", np.ascontiguousarray(rgb[:, :, ::-1]), [int(cv2.IMWRITE_JPEG_QUALITY), q]
    )
    if not ok:
        raise CameraError("JPEG encoding failed")
    return bytes(buf.tobytes())


# ---------------------------------------------------------------------------
# Frame sources
# ---------------------------------------------------------------------------


class OpenCvFrameSource:
    """V4L2 camera through OpenCV (blocking reads, latest-frame buffer of 1)."""

    def __init__(
        self,
        device: str,
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        fps: float = DEFAULT_FPS,
    ) -> None:
        """
        Args:
            device: V4L2 node or stable symlink (``/dev/v4l/by-path/...``).
            width, height, fps: requested capture mode (the camera may pick
                the closest supported one; see :attr:`actual_size`).
        """
        self._device = str(device)
        self._width = int(width)
        self._height = int(height)
        self._fps = float(fps)
        self._cap = None

    @property
    def device(self) -> str:
        """The configured device path."""
        return self._device

    @property
    def is_open(self) -> bool:
        """True while the capture handle is open."""
        return self._cap is not None

    @property
    def actual_size(self) -> Tuple[int, int]:
        """``(width, height)`` reported by the driver (requested if closed)."""
        if self._cap is None:
            return self._width, self._height
        return (
            int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )

    def open(self) -> None:
        """Open the device (idempotent).

        Raises:
            CameraError: OpenCV missing or the device cannot be opened.
        """
        if self._cap is not None:
            return
        if not HAS_CV2:
            raise CameraError("OpenCV (cv2) is not installed")
        cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            raise CameraError(f"cannot open {self._device}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_FPS, self._fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._cap = cap

    def read(self) -> np.ndarray:
        """Block for the next frame and return it as RGB.

        Raises:
            CameraError: not open, or the read failed (the handle is closed so
                the caller's reconnect policy applies).
        """
        if self._cap is None:
            raise CameraError(f"{self._device} is not open")
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self.close()
            raise CameraError(f"read failed on {self._device}")
        return bgr_to_rgb8(frame)

    def close(self) -> None:
        """Release the device, swallowing shutdown errors."""
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:  # pragma: no cover - best-effort shutdown
                pass
            self._cap = None


class MockFrameSource:
    """Deterministic synthetic frames for ``mock`` mode (no camera, no OpenCV).

    A vertical gradient tinted per side with a bright bar that sweeps across
    the image one step per frame. ``realtime`` paces :meth:`read` at ``fps`` so
    a worker thread does not spin.
    """

    def __init__(
        self,
        side: str = "left",
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        fps: float = DEFAULT_FPS,
        realtime: bool = True,
    ) -> None:
        if side not in SIDES:
            raise CameraError(f"unknown side '{side}'")
        self._side = side
        self._width = int(width)
        self._height = int(height)
        self._period_s = 1.0 / max(float(fps), 1e-6)
        self._realtime = bool(realtime)
        self._open = False
        self._count = 0
        ramp = np.linspace(40, 200, self._height, dtype=np.float64)[:, None]
        base = np.repeat(ramp, self._width, axis=1)
        tint = (1.0, 0.55, 0.55) if side == "left" else (0.55, 0.55, 1.0)
        self._base = np.stack([base * t for t in tint], axis=2).astype(np.uint8)

    @property
    def is_open(self) -> bool:
        """True after :meth:`open`."""
        return self._open

    @property
    def actual_size(self) -> Tuple[int, int]:
        """``(width, height)`` of the generated frames."""
        return self._width, self._height

    def open(self) -> None:
        """Mark the source open (never fails)."""
        self._open = True

    def close(self) -> None:
        """Mark the source closed."""
        self._open = False

    def read(self) -> np.ndarray:
        """Next synthetic RGB frame."""
        if not self._open:
            raise CameraError("mock camera is not open")
        if self._realtime:
            time.sleep(self._period_s)
        frame = self._base.copy()
        x = (self._count * 8) % self._width
        frame[:, x : min(x + 8, self._width), :] = 255
        self._count += 1
        return frame


# ---------------------------------------------------------------------------
# Scheduling helpers
# ---------------------------------------------------------------------------


class ReconnectBackoff:
    """Exponential reconnect delay: ``initial``, ``2*initial``, ... capped."""

    def __init__(self, initial_s: float = 2.0, max_s: float = 30.0) -> None:
        self._initial = max(float(initial_s), 0.0)
        self._max = max(float(max_s), self._initial)
        self._next = self._initial

    def next_delay(self) -> float:
        """Delay to wait before the next attempt (then doubles)."""
        delay = self._next
        self._next = min(self._next * 2.0, self._max)
        return delay

    def reset(self) -> None:
        """Back to the initial delay (call after a successful open)."""
        self._next = self._initial


class RateLimiter:
    """Decimate a stream to at most ``rate_hz`` (``<= 0`` passes everything)."""

    def __init__(self, rate_hz: float) -> None:
        self._period = 1.0 / rate_hz if rate_hz > 0.0 else 0.0
        self._next: Optional[float] = None

    def should_publish(self, now_s: float) -> bool:
        """True when a frame arriving at ``now_s`` should be published."""
        if self._period <= 0.0:
            return True
        if self._next is None or now_s >= self._next - 1e-9:
            # Allow for jitter: advance from the schedule, not from "now",
            # unless we fell more than one period behind.
            base = self._next if self._next is not None else now_s
            self._next = base + self._period
            if self._next <= now_s:
                self._next = now_s + self._period
            return True
        return False


class DemandGate:
    """Should an on-demand camera be open? Yes while somebody wants frames, and for ``linger_s`` after the last one.

    The linger keeps a consumer that drops and re-subscribes (the laptop relay unsubscribes 5 s after its last
    subscriber and a viewer may come back) from making the device close and open again each time.
    """

    def __init__(self, linger_s: float = 3.0) -> None:
        self._linger = max(float(linger_s), 0.0)
        self._until: Optional[float] = None

    def wanted(self, now_s: float, demand: bool) -> bool:
        """``demand``: at least one subscriber right now. Call it with the current monotonic time."""
        if demand:
            self._until = now_s + self._linger
            return True
        return self._until is not None and now_s < self._until


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------


@dataclass
class CameraCalibration:
    """Fields of a ``sensor_msgs/CameraInfo``."""

    width: int
    height: int
    distortion_model: str = "plumb_bob"
    d: List[float] = field(default_factory=list)
    k: List[float] = field(default_factory=lambda: [0.0] * 9)
    r: List[float] = field(
        default_factory=lambda: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )
    p: List[float] = field(default_factory=lambda: [0.0] * 12)

    @property
    def calibrated(self) -> bool:
        """True when a camera matrix was provided (``K[0] != 0``, REP-104)."""
        return self.k[0] != 0.0


def uncalibrated(width: int, height: int) -> CameraCalibration:
    """CameraInfo for a camera without calibration (K = P = 0, R = I)."""
    return CameraCalibration(width=int(width), height=int(height))


def _matrix(section: object, name: str, count: int) -> List[float]:
    data = section.get("data") if isinstance(section, dict) else None
    if not isinstance(data, list) or len(data) != count:
        raise CameraError(f"calibration '{name}' must carry {count} values")
    return [float(v) for v in data]


def load_calibration(path: str) -> CameraCalibration:
    """Load a ROS ``camera_calibration`` YAML (``camera_info`` layout).

    Raises:
        CameraError: PyYAML missing, unreadable file or malformed content.
    """
    if not HAS_YAML:
        raise CameraError("PyYAML is required to load a calibration file")
    try:
        with open(path, "r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        raise CameraError(f"cannot read calibration {path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise CameraError(f"calibration {path} is not a mapping")
    try:
        calib = CameraCalibration(
            width=int(raw["image_width"]),
            height=int(raw["image_height"]),
            distortion_model=str(raw.get("distortion_model", "plumb_bob")),
            k=_matrix(raw["camera_matrix"], "camera_matrix", 9),
        )
    except KeyError as exc:
        raise CameraError(f"calibration {path} misses {exc}") from exc
    dist = raw.get("distortion_coefficients")
    if dist is not None:
        data = dist.get("data") if isinstance(dist, dict) else None
        if not isinstance(data, list):
            raise CameraError("calibration 'distortion_coefficients' needs a data list")
        calib.d = [float(v) for v in data]
    if "rectification_matrix" in raw:
        calib.r = _matrix(raw["rectification_matrix"], "rectification_matrix", 9)
    if "projection_matrix" in raw:
        calib.p = _matrix(raw["projection_matrix"], "projection_matrix", 12)
    return calib


def calibration_for_stream(
    calib: Optional[CameraCalibration], width: int, height: int
) -> CameraCalibration:
    """Calibration to publish for a ``width`` x ``height`` stream.

    A calibration recorded at another resolution would give wrong intrinsics,
    so it is dropped in favour of the uncalibrated placeholder.
    """
    if calib is None or (calib.width, calib.height) != (int(width), int(height)):
        return uncalibrated(width, height)
    return calib


FrameFactory = Callable[[str], object]
