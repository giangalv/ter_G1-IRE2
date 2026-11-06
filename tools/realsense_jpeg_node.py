#!/usr/bin/env python3
"""D435i colour as JPEG: runs ON the Jetson (ROS 2 Foxy, DDS domain 42), next to the RealSense driver.

Subscribes /camera/color/image_raw (rgb8, ~66 Mbit/s at 640x360x15) and publishes /camera/color/image_raw/compressed
(~2-3 Mbit/s at 10 Hz, quality 80), so the colour image fits the robot's Wi-Fi (docs/07-wifi.md). It only encodes while someone
is subscribed to the output. Started and stopped with the driver by realsense_ros2_remote.sh.

    python3 realsense_jpeg_node.py [--ros-args -p rate_hz:=10.0 -p quality:=80]
"""
import array
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class ColorJpeg(Node):
    def __init__(self) -> None:
        super().__init__("realsense_color_jpeg")
        self.declare_parameter("in_topic", "/camera/color/image_raw")
        self.declare_parameter("out_topic", "/camera/color/image_raw/compressed")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("quality", 80)
        gp = self.get_parameter
        self._period = 1.0 / max(float(gp("rate_hz").value), 0.5)
        self._quality = int(gp("quality").value)
        self._last = 0.0
        self._pub = self.create_publisher(CompressedImage, str(gp("out_topic").value), qos_profile_sensor_data)
        self.create_subscription(Image, str(gp("in_topic").value), self._on_image, qos_profile_sensor_data)
        self.get_logger().info(f"{gp('in_topic').value} -> {gp('out_topic').value}, <= {1.0 / self._period:g} Hz, quality {self._quality}")

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last < self._period:
            return
        try:
            if self._pub.get_subscription_count() == 0:
                return  # nobody listens: do not even encode
        except AttributeError:
            pass
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warning(f"colour encoding {msg.encoding!r}, expected rgb8/bgr8", throttle_duration_sec=10.0)
            return
        self._last = now
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self._quality])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = array.array("B", buf.tobytes())
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ColorJpeg()
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
