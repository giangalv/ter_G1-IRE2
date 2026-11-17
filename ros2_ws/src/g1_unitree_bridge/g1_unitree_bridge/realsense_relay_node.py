"""Copy the D435i colour and depth from the RealSense driver's DDS domain to the robot's domain + make a cloud.

The RealSense ROS 2 driver runs on the Jetson (ROS 2 Foxy, Fast DDS) in its OWN DDS domain
(``rs_domain``, default 42): in domain 0 the Fast DDS of Foxy dies parsing the discovery data of the
robot's participants (bad_alloc), and Foxy's Cyclone segfaults. This node opens two contexts:

    domain ``rs_domain``  /camera/depth/{image_rect_raw (16UC1, mm), camera_info}, /camera/color/{image_raw, camera_info}
    robot domain (ROS_DOMAIN_ID, normally 0)
        /g1/camera/depth/image        32FC1, metres (CONVENTIONS.md), NaN = no measurement
        /g1/camera/depth/camera_info
        /g1/camera/depth/points       XYZ cloud in ``frame_id`` (optical frame), every ``stride``-th pixel
        /g1/camera/color/image        rgb8, rate limited
        /g1/camera/color/camera_info

It only receives from the camera domain and publishes to the robot domain.

Two options for the robot's Wi-Fi (docs/07-wifi.md in the base repo):
  * ``lazy`` (default true): a camera stream is only subscribed to in the RealSense domain while somebody
    subscribes to its outputs here (RViz display on/off = traffic on/off); it is dropped 5 s after the last
    subscriber is gone.
  * ``color_compressed`` (default false): take the colour as JPEG (``/camera/color/image_raw/compressed``,
    made on the Jetson by tools/realsense_encoder_node.py, ~1.5 Mbit/s) instead of raw (~66 Mbit/s), decode it
    here and publish the same ``/g1/camera/color/image`` as before.
  * ``depth_compressed`` (default false): the same for the depth, as a lossless 16-bit PNG
    (``/camera/depth/image_rect_raw/compressedDepth``, image_transport format) at the rate the Jetson's encoder is set to
    (default 5 Hz), instead of raw (203 kB per frame, ~24 Mbit/s at 15 Hz). The images and the cloud published here are
    the same either way.
"""
from __future__ import annotations

import time
from typing import Callable, Dict, List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField

from g1_unitree_bridge.depth_core import decode_compressed_depth, depth_to_metres, depth_to_points, intrinsics


class RealSenseRelay(Node):
    """Output side (robot domain); the input node lives in a second context."""

    def __init__(self) -> None:
        super().__init__("realsense_relay")
        self.declare_parameter("rs_domain", 42)
        self.declare_parameter("in_prefix", "/camera/depth")
        self.declare_parameter("frame_id", "d435_depth_optical_frame")
        self.declare_parameter("color_prefix", "/camera/color")
        self.declare_parameter("color_frame_id", "d435_color_optical_frame")
        self.declare_parameter("color_rate_hz", 10.0)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("image_rate_hz", 10.0)
        self.declare_parameter("points_rate_hz", 5.0)
        self.declare_parameter("stride", 2)
        self.declare_parameter("max_range", 6.0)
        self.declare_parameter("lazy", True)
        self.declare_parameter("color_compressed", False)
        self.declare_parameter("depth_compressed", False)
        # true (default): stamp every output with THIS PC's clock. The Jetson's clock is not synchronised (tens of seconds off, measured
        # 2026-11-13), and RViz drops a cloud whose stamp is older than its TF buffer.
        self.declare_parameter("restamp", True)

        gp = self.get_parameter
        self._frame = str(gp("frame_id").value)
        self._scale = float(gp("depth_scale").value)
        self._restamp = bool(gp("restamp").value)
        self._stride = int(gp("stride").value)
        self._max_range = float(gp("max_range").value)
        self._image_period = 1.0 / max(float(gp("image_rate_hz").value), 0.1)
        self._points_period = 1.0 / max(float(gp("points_rate_hz").value), 0.1)
        self._color_frame = str(gp("color_frame_id").value)
        self._color_period = 1.0 / max(float(gp("color_rate_hz").value), 0.1)
        self._last_color = 0.0
        self._last_image = 0.0
        self._last_points = 0.0
        self._k: Optional[tuple] = None

        self._image_pub = self.create_publisher(Image, "/g1/camera/depth/image", qos_profile_sensor_data)
        self._info_pub = self.create_publisher(CameraInfo, "/g1/camera/depth/camera_info", qos_profile_sensor_data)
        self._points_pub = self.create_publisher(PointCloud2, "/g1/camera/depth/points", qos_profile_sensor_data)
        self._color_pub = self.create_publisher(Image, "/g1/camera/color/image", qos_profile_sensor_data)
        self._color_info_pub = self.create_publisher(CameraInfo, "/g1/camera/color/camera_info", qos_profile_sensor_data)

        # second context = the RealSense domain
        self._in_ctx = Context()
        rclpy.init(context=self._in_ctx, domain_id=int(gp("rs_domain").value))
        self._in_node = rclpy.create_node("realsense_relay_in", context=self._in_ctx)
        prefix = str(gp("in_prefix").value)
        color = str(gp("color_prefix").value)
        self._lazy = bool(gp("lazy").value)
        color_compressed = bool(gp("color_compressed").value)
        depth_compressed = bool(gp("depth_compressed").value)
        # stream groups: (outputs that must have a subscriber, input subscriptions)
        color_specs = [(CameraInfo, f"{color}/camera_info", self._on_color_info)]
        if color_compressed:
            color_specs.append((CompressedImage, f"{color}/image_raw/compressed", self._on_color_jpeg))
        else:
            color_specs.append((Image, f"{color}/image_raw", self._on_color))
        self._groups: Dict[str, dict] = {
            "depth": {
                "outputs": (self._image_pub, self._info_pub, self._points_pub),
                "specs": [
                    (CameraInfo, f"{prefix}/camera_info", self._on_info),
                    (CompressedImage, f"{prefix}/image_rect_raw/compressedDepth", self._on_depth_png)
                    if depth_compressed
                    else (Image, f"{prefix}/image_rect_raw", self._on_depth),
                ],
                "subs": [],
                "idle_since": None,
            },
            "color": {
                "outputs": (self._color_pub, self._color_info_pub),
                "specs": color_specs,
                "subs": [],
                "idle_since": None,
            },
        }
        self._next_check = 0.0
        self._executor = SingleThreadedExecutor(context=self._in_ctx)
        self._executor.add_node(self._in_node)
        self.get_logger().info(
            f"realsense_relay up: domain {gp('rs_domain').value} {prefix} -> /g1/camera/depth/{{image,camera_info,points}} "
            f"(cloud every {self._stride}th pixel, <= {self._max_range:g} m); colour {gp('color_prefix').value}/image_raw"
            f"{'/compressed' if color_compressed else ''} -> /g1/camera/color/image; depth "
            f"{'compressedDepth (PNG)' if depth_compressed else 'raw'}; lazy={self._lazy}"
        )

    def spin_input(self) -> None:
        while rclpy.ok(context=self._in_ctx) and rclpy.ok():
            self._update_groups()
            self._executor.spin_once(timeout_sec=0.2)

    def _update_groups(self) -> None:
        """Subscribe a stream in the camera domain while its outputs have subscribers (all the time when not lazy)."""
        now = time.monotonic()
        if now < self._next_check:
            return
        self._next_check = now + 1.0
        for name, group in self._groups.items():
            wanted = (not self._lazy) or any(pub.get_subscription_count() > 0 for pub in group["outputs"])
            if wanted:
                group["idle_since"] = None
                if not group["subs"]:
                    group["subs"] = [
                        self._in_node.create_subscription(msg_type, topic, cb, qos_profile_sensor_data)
                        for msg_type, topic, cb in group["specs"]
                    ]
                    self.get_logger().info(f"{name}: someone is listening, subscribed in the camera domain")
            elif group["subs"]:
                if group["idle_since"] is None:
                    group["idle_since"] = now
                elif now - group["idle_since"] > 5.0:
                    for sub in group["subs"]:
                        self._in_node.destroy_subscription(sub)
                    group["subs"] = []
                    group["idle_since"] = None
                    self.get_logger().info(f"{name}: no subscriber for 5 s, unsubscribed in the camera domain")

    def _on_info(self, msg: CameraInfo) -> None:
        try:
            self._k = intrinsics(tuple(msg.k))
        except ValueError:
            return
        msg.header.frame_id = self._frame
        self._info_pub.publish(msg)

    def _on_color_info(self, msg: CameraInfo) -> None:
        msg.header.frame_id = self._color_frame
        self._color_info_pub.publish(msg)

    def _on_color(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_color < self._color_period:
            return
        self._last_color = now
        msg.header.frame_id = self._color_frame
        self._color_pub.publish(msg)

    def _on_color_jpeg(self, msg: CompressedImage) -> None:
        now = time.monotonic()
        if now - self._last_color < self._color_period:
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("colour JPEG could not be decoded", throttle_duration_sec=10.0)
            return
        self._last_color = now
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        out = Image()
        out.header = msg.header
        out.header.frame_id = self._color_frame
        if self._restamp:
            out.header.stamp = self.get_clock().now().to_msg()
        out.height, out.width = int(rgb.shape[0]), int(rgb.shape[1])
        out.encoding = "rgb8"
        out.step = out.width * 3
        out.data = rgb.tobytes()
        self._color_pub.publish(out)

    def _depth_due(self, now: float) -> tuple:
        return (
            now - self._last_image >= self._image_period,
            self._k is not None and now - self._last_points >= self._points_period,
        )

    def _on_depth(self, msg: Image) -> None:
        if msg.encoding != "16UC1":
            self.get_logger().warning(f"depth encoding {msg.encoding!r}, expected 16UC1", throttle_duration_sec=10.0)
            return
        now = time.monotonic()
        want_image, want_points = self._depth_due(now)
        if not (want_image or want_points):
            return
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        self._publish_depth(msg.header, raw, now, want_image, want_points)

    def _on_depth_png(self, msg: CompressedImage) -> None:
        now = time.monotonic()
        want_image, want_points = self._depth_due(now)
        if not (want_image or want_points):
            return  # do not decode what would be dropped
        raw = decode_compressed_depth(bytes(msg.data))
        if raw is None:
            self.get_logger().warning(f"depth PNG could not be decoded (format {msg.format!r})", throttle_duration_sec=10.0)
            return
        self._publish_depth(msg.header, raw, now, want_image, want_points)

    def _publish_depth(self, header, raw: np.ndarray, now: float, want_image: bool, want_points: bool) -> None:
        metres = depth_to_metres(raw, self._scale)
        if want_image:
            self._last_image = now
            out = Image()
            out.header = header
            out.header.frame_id = self._frame
            if self._restamp:
                out.header.stamp = self.get_clock().now().to_msg()
            out.height, out.width = int(raw.shape[0]), int(raw.shape[1])
            out.encoding = "32FC1"
            out.step = out.width * 4
            out.data = metres.tobytes()
            self._image_pub.publish(out)
        if want_points:
            self._last_points = now
            pts = depth_to_points(metres, self._k, self._stride, self._max_range)
            cloud = PointCloud2()
            cloud.header = header
            cloud.header.frame_id = self._frame
            if self._restamp:
                cloud.header.stamp = self.get_clock().now().to_msg()
            cloud.height, cloud.width = 1, int(pts.shape[0])
            cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
            cloud.is_bigendian = False
            cloud.point_step = 12
            cloud.row_step = 12 * cloud.width
            cloud.is_dense = True
            cloud.data = pts.tobytes()
            self._points_pub.publish(cloud)

    def destroy_node(self) -> None:
        self._executor.shutdown()
        self._in_node.destroy_node()
        rclpy.shutdown(context=self._in_ctx)
        super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = RealSenseRelay()
    try:
        node.spin_input()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
