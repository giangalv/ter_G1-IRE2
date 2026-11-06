#!/usr/bin/env python3
"""D435i streams compressed for the robot's Wi-Fi: runs ON the Jetson (ROS 2 Foxy, DDS domain 42), next to the driver.

    /camera/color/image_raw        rgb8 640x360x15  ~66 Mbit/s  ->  /camera/color/image_raw/compressed             JPEG (lossy)
    /camera/depth/image_rect_raw   16UC1 424x240x15 ~24 Mbit/s  ->  /camera/depth/image_rect_raw/compressedDepth    PNG 16 bit (lossless)

Each output is only produced while somebody subscribes to it, and at most ``*_rate_hz`` frames per second. The depth
topic uses the image_transport "compressedDepth" layout (format "16UC1; compressedDepth png", 12-byte header, PNG). It is
lossless (checked frame by frame against the raw topic) and decoded by g1_unitree_bridge/depth_core.py; image_transport's own
Humble `republish` tool fails with this plugin ("subscribeImpl with five arguments has not been overridden"), so it was not
verified with it. The PC side (g1_unitree_bridge/realsense_relay) chooses raw or compressed. Started and stopped with the
driver by realsense_ros2_remote.sh.

All parameters can be changed while it runs, from the laptop: ``tools/realsense_ros2.sh stream depth_rate_hz=8``
(on the Jetson: ``ros2 param set /realsense_encoder depth_rate_hz 8.0``). docs/07-wifi.md says which values to pick.

    python3 realsense_encoder_node.py [--ros-args -p depth_rate_hz:=5.0 -p color_quality:=80 ...]
"""
import array
import struct
import time

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

# image_transport compressedDepth: int32 format (0 = INV_DEPTH, unused for 16UC1) + float depthParam[2], then the PNG
DEPTH_HEADER = struct.pack("<iff", 0, 0.0, 0.0)


class Throttle:
    """At most ``hz`` frames per second without drifting (a 15 Hz source at 5 Hz gives every third frame)."""

    def __init__(self, hz: float) -> None:
        self.set_rate(hz)
        self._next = 0.0

    def set_rate(self, hz: float) -> None:
        self._period = 1.0 / max(float(hz), 0.2)

    def due(self, now: float) -> bool:
        if now + 0.004 < self._next:
            return False
        self._next = self._next + self._period if now - self._next < self._period else now + self._period
        return True


class Encoder(Node):
    def __init__(self) -> None:
        super().__init__("realsense_encoder")
        defaults = {
            "color_in": "/camera/color/image_raw",
            "color_out": "/camera/color/image_raw/compressed",
            "color_rate_hz": 10.0,
            "color_quality": 80,
            "depth_in": "/camera/depth/image_rect_raw",
            "depth_out": "/camera/depth/image_rect_raw/compressedDepth",
            "depth_rate_hz": 5.0,
            "depth_png_level": 1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        gp = self.get_parameter
        self._color_gate = Throttle(float(gp("color_rate_hz").value))
        self._depth_gate = Throttle(float(gp("depth_rate_hz").value))
        self._quality = int(gp("color_quality").value)
        self._png_level = int(gp("depth_png_level").value)
        self.add_on_set_parameters_callback(self._on_params)
        self._color_pub = self.create_publisher(CompressedImage, str(gp("color_out").value), qos_profile_sensor_data)
        self._depth_pub = self.create_publisher(CompressedImage, str(gp("depth_out").value), qos_profile_sensor_data)
        self.create_subscription(Image, str(gp("color_in").value), self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, str(gp("depth_in").value), self._on_depth, qos_profile_sensor_data)
        self.get_logger().info(
            f"colour JPEG <= {gp('color_rate_hz').value} Hz q{self._quality}; depth PNG level {self._png_level} <= {gp('depth_rate_hz').value} Hz"
        )

    def _on_params(self, params):
        for p in params:
            if p.name == "color_rate_hz":
                self._color_gate.set_rate(p.value)
            elif p.name == "depth_rate_hz":
                self._depth_gate.set_rate(p.value)
            elif p.name == "color_quality":
                self._quality = max(1, min(100, int(p.value)))
            elif p.name == "depth_png_level":
                self._png_level = max(0, min(9, int(p.value)))
        return SetParametersResult(successful=True)

    @staticmethod
    def _listening(pub) -> bool:
        try:
            return pub.get_subscription_count() > 0
        except AttributeError:
            return True

    def _on_color(self, msg: Image) -> None:
        if not self._listening(self._color_pub) or not self._color_gate.due(time.monotonic()):
            return  # nobody listens, or not due yet: do not even encode
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warning(f"colour encoding {msg.encoding!r}, expected rgb8/bgr8", throttle_duration_sec=10.0)
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self._quality])
        if ok:
            self._publish(self._color_pub, msg, "jpeg", buf.tobytes())

    def _on_depth(self, msg: Image) -> None:
        if not self._listening(self._depth_pub) or not self._depth_gate.due(time.monotonic()):
            return
        if msg.encoding != "16UC1":
            self.get_logger().warning(f"depth encoding {msg.encoding!r}, expected 16UC1", throttle_duration_sec=10.0)
            return
        img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        ok, buf = cv2.imencode(".png", img, [int(cv2.IMWRITE_PNG_COMPRESSION), self._png_level])
        if ok:
            self._publish(self._depth_pub, msg, "16UC1; compressedDepth png", DEPTH_HEADER + buf.tobytes())

    @staticmethod
    def _publish(pub, src: Image, fmt: str, data: bytes) -> None:
        out = CompressedImage()
        out.header = src.header
        out.format = fmt
        out.data = array.array("B", data)
        pub.publish(out)


def main() -> None:
    rclpy.init()
    node = Encoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
