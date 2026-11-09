"""``hand_camera_node``: palm cameras of the Inspire hands -> ROS 2 images.

Thin ROS 2 shell around :mod:`g1_hand_camera.camera_core`. One capture thread
per hand (OpenCV ``read`` blocks), the ROS side only publishes:

* ``/g1/hand/{left,right}/camera/image_raw`` (``sensor_msgs/Image``, ``rgb8``)
* ``/g1/hand/{left,right}/camera/image_raw/compressed`` (JPEG)
* ``/g1/hand/{left,right}/camera/camera_info``
* ``/g1/hand/{left,right}/camera/available`` (latched ``Bool``)

All streams use the sensor QoS (BEST_EFFORT); ``available`` is latched like
``/g1/hand/*/available``. Each representation is encoded/published only while
somebody subscribes to it, so an idle node costs one USB read per frame.

Hot-plug: when a camera cannot be opened or a read fails the worker marks the
camera unavailable and retries with exponential backoff (2 s -> 30 s).

On demand (``on_demand=True``): a camera is opened only while somebody subscribes to one of its topics and for
``linger_s`` after the last subscriber is gone; otherwise the device stays closed (no USB traffic, nothing to
reopen when the USB hub drops the cameras, e.g. while the head camera restarts). ``available`` then means "the
last open worked": closing on purpose does not clear it, a failed open or read does. The first frame comes about
a second after the first subscriber.

Mock mode (``mock=True``): synthetic frames, no OpenCV and no device needed.
If ``mock`` is False and OpenCV is missing, startup fails fatally
(CONVENTIONS.md mock rules).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool

from g1_hand_camera.camera_core import (
    DEFAULT_DEVICES,
    DEFAULT_FPS,
    DEFAULT_HEIGHT,
    DEFAULT_WIDTH,
    HAS_CV2,
    SIDES,
    CameraCalibration,
    CameraError,
    DemandGate,
    MockFrameSource,
    OpenCvFrameSource,
    RateLimiter,
    ReconnectBackoff,
    calibration_for_stream,
    encode_jpeg,
    load_calibration,
    to_u8_array,
)
from g1_hand_camera.qos import LATCHED_QOS, SENSOR_QOS

_STATS_PERIOD_S = 10.0
_JOIN_TIMEOUT_S = 2.0
_IDLE_POLL_S = 0.2  # on demand: how often a closed camera looks for subscribers


@dataclass
class _CameraState:
    """Node-side bookkeeping for one palm camera."""

    side: str
    device: str
    frame_id: str
    source: object
    calibration: CameraCalibration
    raw_pub: object = None
    comp_pub: object = None
    info_pub: object = None
    avail_pub: object = None
    thread: Optional[threading.Thread] = None
    available: Optional[bool] = None  # None = nothing published yet
    frames: int = 0
    published: int = 0
    errors: int = 0
    limiter: Optional[RateLimiter] = None
    gate: Optional[DemandGate] = None
    lock: threading.Lock = field(default_factory=threading.Lock)


class HandCameraNode(Node):
    """Dual palm-camera publisher (one worker thread per hand)."""

    def __init__(self) -> None:
        super().__init__("hand_camera_node")

        # -- parameters ------------------------------------------------------
        self.declare_parameter("device_left", DEFAULT_DEVICES["left"])
        self.declare_parameter("device_right", DEFAULT_DEVICES["right"])
        self.declare_parameter("width", DEFAULT_WIDTH)
        self.declare_parameter("height", DEFAULT_HEIGHT)
        self.declare_parameter("capture_fps", DEFAULT_FPS)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("publish_raw", True)
        self.declare_parameter("publish_compressed", True)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("calibration_left", "")
        self.declare_parameter("calibration_right", "")
        self.declare_parameter("frame_id_left", "left_hand_camera")
        self.declare_parameter("frame_id_right", "right_hand_camera")
        self.declare_parameter("reconnect_initial_s", 2.0)
        self.declare_parameter("reconnect_max_s", 30.0)
        self.declare_parameter("on_demand", False)
        self.declare_parameter("linger_s", 3.0)
        self.declare_parameter("mock", False)

        gp = self.get_parameter
        self._mock = bool(gp("mock").value)
        self._width = int(gp("width").value)
        self._height = int(gp("height").value)
        self._fps = float(gp("capture_fps").value)
        self._rate_hz = float(gp("publish_rate_hz").value)
        self._publish_raw = bool(gp("publish_raw").value)
        self._publish_comp = bool(gp("publish_compressed").value)
        self._jpeg_quality = int(gp("jpeg_quality").value)
        self._reconnect_initial_s = float(gp("reconnect_initial_s").value)
        self._reconnect_max_s = float(gp("reconnect_max_s").value)
        self._on_demand = bool(gp("on_demand").value)
        linger_s = float(gp("linger_s").value)

        if not self._mock and not HAS_CV2:
            self.get_logger().fatal(
                "OpenCV (cv2) is not installed and mock=False; refusing to "
                "start (set mock:=true for sim mode)"
            )
            raise RuntimeError("cv2 missing and mock=False")

        # -- per-hand state --------------------------------------------------
        self._stop = threading.Event()
        self._cams: Dict[str, _CameraState] = {}
        for side in SIDES:
            device = str(gp(f"device_{side}").value)
            calib = self._load_calibration(side, str(gp(f"calibration_{side}").value))
            if self._mock:
                source = MockFrameSource(side, self._width, self._height, self._fps)
            else:
                source = OpenCvFrameSource(device, self._width, self._height, self._fps)
            cam = _CameraState(
                side=side,
                device="mock" if self._mock else device,
                frame_id=str(gp(f"frame_id_{side}").value),
                source=source,
                calibration=calib,
                limiter=RateLimiter(self._rate_hz),
                gate=DemandGate(linger_s),
            )
            prefix = f"/g1/hand/{side}/camera"
            cam.raw_pub = self.create_publisher(Image, f"{prefix}/image_raw", SENSOR_QOS)
            cam.comp_pub = self.create_publisher(
                CompressedImage, f"{prefix}/image_raw/compressed", SENSOR_QOS
            )
            cam.info_pub = self.create_publisher(CameraInfo, f"{prefix}/camera_info", SENSOR_QOS)
            cam.avail_pub = self.create_publisher(Bool, f"{prefix}/available", LATCHED_QOS)
            self._cams[side] = cam
            self._set_available(cam, False)

        for cam in self._cams.values():
            cam.thread = threading.Thread(
                target=self._capture_loop,
                args=(cam,),
                name=f"hand_camera_{cam.side}",
                daemon=True,
            )
            cam.thread.start()

        self._stats_timer = self.create_timer(_STATS_PERIOD_S, self._log_stats)
        self.get_logger().info(
            f"hand_camera_node up (mock={self._mock}, {self._width}x{self._height}"
            f"@{self._fps:.0f}, publish {self._rate_hz:g} Hz, "
            + (f"on demand, linger {linger_s:g} s" if self._on_demand else "always open")
            + "): "
            + ", ".join(f"{c.side}={c.device}" for c in self._cams.values())
        )

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------

    def _load_calibration(self, side: str, path: str) -> CameraCalibration:
        """Calibration for ``side`` (uncalibrated placeholder on any problem)."""
        loaded: Optional[CameraCalibration] = None
        if path:
            try:
                loaded = load_calibration(path)
            except CameraError as exc:
                self.get_logger().error(f"{side}: {exc}; publishing uncalibrated CameraInfo")
            else:
                if (loaded.width, loaded.height) != (self._width, self._height):
                    self.get_logger().warning(
                        f"{side}: calibration is {loaded.width}x{loaded.height} but the "
                        f"stream is {self._width}x{self._height}; publishing uncalibrated "
                        "CameraInfo"
                    )
        else:
            self.get_logger().info(f"{side}: no calibration file, CameraInfo is uncalibrated")
        return calibration_for_stream(loaded, self._width, self._height)

    def _running(self) -> bool:
        """False once shutdown was requested (also by the rclpy signal handler)."""
        return not self._stop.is_set() and rclpy.ok()

    def _set_available(self, cam: _CameraState, value: bool) -> None:
        """Publish the latched availability flag when it changes."""
        if not rclpy.ok():  # context already shut down by SIGINT/SIGTERM
            self._stop.set()
            return
        with cam.lock:
            if cam.available is value:
                return
            cam.available = value
        cam.avail_pub.publish(Bool(data=value))
        if value:
            self.get_logger().info(f"{cam.side} palm camera available ({cam.device})")
        else:
            self.get_logger().warning(f"{cam.side} palm camera unavailable ({cam.device})")

    # ------------------------------------------------------------------
    # Capture thread
    # ------------------------------------------------------------------

    def _wanted(self, cam: _CameraState) -> bool:
        """On demand: is somebody subscribed to this camera (or was somebody within the linger time)?"""
        demand = (
            cam.raw_pub.get_subscription_count() > 0
            or cam.comp_pub.get_subscription_count() > 0
            or cam.info_pub.get_subscription_count() > 0
        )
        return cam.gate.wanted(time.monotonic(), demand)

    def _capture_loop(self, cam: _CameraState) -> None:
        """Open / read / publish loop of one camera, with hot-plug retry."""
        backoff = ReconnectBackoff(self._reconnect_initial_s, self._reconnect_max_s)
        while self._running():
            if self._on_demand and not self._wanted(cam):
                self._stop.wait(_IDLE_POLL_S)  # closed: nobody listens
                continue
            try:
                cam.source.open()
            except CameraError as exc:
                cam.errors += 1
                delay = backoff.next_delay()
                self.get_logger().warning(
                    f"{cam.side}: {exc}; retry in {delay:.0f} s",
                    throttle_duration_sec=10.0,
                )
                self._stop.wait(delay)
                continue

            backoff.reset()
            width, height = cam.source.actual_size
            if (width, height) != (self._width, self._height):
                self.get_logger().warning(
                    f"{cam.side}: driver gave {width}x{height}, requested "
                    f"{self._width}x{self._height}",
                    throttle_duration_sec=10.0,
                )
            self._set_available(cam, True)

            closed_on_purpose = False
            while self._running():
                if self._on_demand and not self._wanted(cam):
                    closed_on_purpose = True
                    break
                try:
                    rgb = cam.source.read()
                except CameraError as exc:
                    cam.errors += 1
                    self.get_logger().warning(f"{cam.side}: {exc}")
                    break
                cam.frames += 1
                try:
                    self._publish_frame(cam, rgb)
                except Exception as exc:  # keep capturing whatever happens
                    cam.errors += 1
                    self.get_logger().error(
                        f"{cam.side}: publish failed: {exc}", throttle_duration_sec=5.0
                    )
            cam.source.close()
            if closed_on_purpose:
                continue  # nothing is wrong with the camera, it just has no audience
            self._set_available(cam, False)
            self._stop.wait(backoff.next_delay())
        cam.source.close()

    def _publish_frame(self, cam: _CameraState, rgb) -> None:
        """Publish one RGB frame in every representation somebody listens to."""
        want_raw = self._publish_raw and cam.raw_pub.get_subscription_count() > 0
        want_comp = self._publish_comp and cam.comp_pub.get_subscription_count() > 0
        want_info = cam.info_pub.get_subscription_count() > 0
        if not (want_raw or want_comp or want_info):
            return
        if not cam.limiter.should_publish(time.monotonic()):
            return

        stamp = self.get_clock().now().to_msg()
        height, width = int(rgb.shape[0]), int(rgb.shape[1])

        if want_raw:
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = cam.frame_id
            msg.height = height
            msg.width = width
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = width * 3
            msg.data = to_u8_array(rgb.tobytes())
            cam.raw_pub.publish(msg)
        if want_comp:
            comp = CompressedImage()
            comp.header.stamp = stamp
            comp.header.frame_id = cam.frame_id
            comp.format = "jpeg"
            comp.data = to_u8_array(encode_jpeg(rgb, self._jpeg_quality))
            cam.comp_pub.publish(comp)
        if want_info:
            cam.info_pub.publish(self._camera_info(cam, stamp, width, height))
        cam.published += 1

    @staticmethod
    def _camera_info(cam: _CameraState, stamp, width: int, height: int) -> CameraInfo:
        """CameraInfo for the current frame (uncalibrated when K is zero)."""
        calib = cam.calibration
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = cam.frame_id
        info.width = width
        info.height = height
        info.distortion_model = calib.distortion_model
        info.d = [float(v) for v in calib.d]
        info.k = [float(v) for v in calib.k]
        info.r = [float(v) for v in calib.r]
        info.p = [float(v) for v in calib.p]
        return info

    # ------------------------------------------------------------------
    # Diagnostics / shutdown
    # ------------------------------------------------------------------

    def _log_stats(self) -> None:
        """Periodic frame/publish counters (also a liveness hint in logs)."""
        parts: List[str] = []
        for cam in self._cams.values():
            parts.append(
                f"{cam.side}: avail={cam.available} read={cam.frames} "
                f"published={cam.published} errors={cam.errors}"
            )
        self.get_logger().info("hand cameras | " + " | ".join(parts))

    def shutdown(self) -> None:
        """Stop the capture threads and release the devices."""
        self._stop.set()
        for cam in self._cams.values():
            if cam.thread is not None:
                cam.thread.join(timeout=_JOIN_TIMEOUT_S)


def main(args: Optional[List[str]] = None) -> None:
    """Entry point: spin the node until interrupted."""
    rclpy.init(args=args)
    node = HandCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
