"""Head camera (RealSense D435i colour) from Unitree's multicast video to ``/g1/camera/color/image``.

On the Jetson, Unitree's ``videohub_pc4`` reads the D435i colour node, encodes it to H.264 and
sends it as RTP to the multicast group 230.1.1.1:1720 on the robot LAN (the phone app shows the
same stream; seen on 2026-11-05: 1280x720, ~15 fps, ~8 Mbit/s). This node joins the group, decodes it with GStreamer (through OpenCV) and
publishes ``sensor_msgs/Image`` (rgb8). It only receives: nothing is sent to the robot.

If no picture arrives, the sender is probably not running or still pinned to the wrong NIC after a
boot: see ``tools/start_app_video.sh`` in the base repo (docs/01-network-and-computers.md).
This is the colour image only; depth and the D435i IMU are not part of this stream.
"""
from __future__ import annotations

import threading
import time
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def gst_pipeline(address: str, port: int, iface: str) -> str:
    """GStreamer description of the RTP/H.264 receiver ending in an appsink of BGR frames."""
    iface_opt = f" multicast-iface={iface}" if iface else ""
    return (
        f"udpsrc address={address} port={port}{iface_opt} "
        'caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" '
        "! rtpjitterbuffer latency=100 ! rtph264depay ! avdec_h264 "
        # no h264parse: it is in gst-plugins-bad, not installed here, and avdec_h264 does not need it
        "! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
    )


class HeadVideo(Node):
    """Decode the head video in a thread, publish at most ``max_rate_hz`` frames per second."""

    def __init__(self) -> None:
        super().__init__("head_video")
        self.declare_parameter("address", "230.1.1.1")
        self.declare_parameter("port", 1720)
        self.declare_parameter("iface", "")  # NIC that faces the robot LAN (empty = system default)
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("scale", 0.5)  # 1280x720 -> 640x360; 1.0 loses most frames (2.7 MB samples)
        self.declare_parameter("frame_id", "d435_link")
        self.declare_parameter("topic", "/g1/camera/color/image")
        self.declare_parameter("stall_s", 4.0)  # no frame for this long -> reopen the receiver

        gp = self.get_parameter
        self._pipeline = gst_pipeline(str(gp("address").value), int(gp("port").value), str(gp("iface").value))
        self._period = 1.0 / max(float(gp("max_rate_hz").value), 0.1)
        self._scale = float(gp("scale").value)
        self._frame_id = str(gp("frame_id").value)
        self._stall_s = float(gp("stall_s").value)
        self._pub = self.create_publisher(Image, str(gp("topic").value), qos_profile_sensor_data)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"head_video up: {gp('address').value}:{gp('port').value} via '{gp('iface').value}' -> "
            f"{gp('topic').value} (max {1.0 / self._period:g} Hz, scale {self._scale:g})"
        )

    def _run(self) -> None:
        announced = False
        while not self._stop.is_set():
            cap = cv2.VideoCapture(self._pipeline, cv2.CAP_GSTREAMER)
            if not cap.isOpened():
                self.get_logger().error("cannot open the GStreamer receiver", throttle_duration_sec=10.0)
                self._stop.wait(2.0)
                continue
            last_frame = time.monotonic()
            last_pub = 0.0
            while not self._stop.is_set():
                ok, frame = cap.read()
                now = time.monotonic()
                if not ok:
                    if now - last_frame > self._stall_s:
                        break
                    continue
                last_frame = now
                if not announced:
                    self.get_logger().info(f"first frame {frame.shape[1]}x{frame.shape[0]}")
                    announced = True
                if now - last_pub < self._period:
                    continue
                last_pub = now
                self._publish(frame)
            cap.release()
            if not self._stop.is_set():
                self.get_logger().warning(
                    f"no video for {self._stall_s:g} s: is the sender running? reopening",
                    throttle_duration_sec=10.0,
                )
                announced = False

    def _publish(self, bgr: np.ndarray) -> None:
        if self._scale != 1.0:
            bgr = cv2.resize(bgr, None, fx=self._scale, fy=self._scale, interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height, msg.width = int(rgb.shape[0]), int(rgb.shape[1])
        msg.encoding = "rgb8"
        msg.step = msg.width * 3
        msg.data = rgb.tobytes()
        self._pub.publish(msg)

    def destroy_node(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)
        super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = HeadVideo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
