"""Copy the palm cameras (JPEG) from the Jetson's DDS domain to the robot's domain.

``g1_hand_camera/hand_camera_node`` runs on the Jetson (ROS 2 Foxy, Fast DDS) in its OWN DDS domain
(``rs_domain``, default 42), like the RealSense driver: Foxy's DDS crashes in the robot's domain 0.
This node opens two contexts and only receives from the camera domain:

    domain ``rs_domain``  /g1/hand/{left,right}/camera/image_raw/compressed, .../camera/available
    robot domain (ROS_DOMAIN_ID, normally 0)  the same topic names, same types

Frames are rate limited to ``max_rate_hz`` (0 = every frame). ``available`` is latched on both sides.
``lazy`` (default true, for the robot's Wi-Fi): a side's JPEG stream is only subscribed to in the camera domain while
somebody subscribes to its output here, and dropped 5 s after the last one is gone.
"""
from __future__ import annotations

import time
from typing import Dict, List, Optional

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

SIDES = ("left", "right")
LATCHED = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class HandCameraRelay(Node):
    """Output side (robot domain); the input node lives in a second context."""

    def __init__(self) -> None:
        super().__init__("hand_camera_relay")
        self.declare_parameter("rs_domain", 42)
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("lazy", True)
        gp = self.get_parameter
        rate = float(gp("max_rate_hz").value)
        self._period = 1.0 / rate if rate > 0.0 else 0.0
        self._last: Dict[str, float] = {s: 0.0 for s in SIDES}
        self._jpeg_pub = {
            s: self.create_publisher(CompressedImage, f"/g1/hand/{s}/camera/image_raw/compressed", qos_profile_sensor_data)
            for s in SIDES
        }
        self._avail_pub = {s: self.create_publisher(Bool, f"/g1/hand/{s}/camera/available", LATCHED) for s in SIDES}

        self._in_ctx = Context()
        rclpy.init(context=self._in_ctx, domain_id=int(gp("rs_domain").value))
        self._in_node = rclpy.create_node("hand_camera_relay_in", context=self._in_ctx)
        self._lazy = bool(gp("lazy").value)
        self._jpeg_sub: Dict[str, object] = {}
        self._idle_since: Dict[str, Optional[float]] = {s: None for s in SIDES}
        self._next_check = 0.0
        for s in SIDES:
            self._in_node.create_subscription(Bool, f"/g1/hand/{s}/camera/available", self._avail_cb(s), LATCHED)
        self._update_streams()
        self._executor = SingleThreadedExecutor(context=self._in_ctx)
        self._executor.add_node(self._in_node)
        self.get_logger().info(
            f"hand_camera_relay up: domain {gp('rs_domain').value} /g1/hand/{{left,right}}/camera/image_raw/compressed "
            f"-> robot domain (<= {rate:g} Hz)"
        )

    def spin_input(self) -> None:
        while rclpy.ok(context=self._in_ctx) and rclpy.ok():
            self._update_streams()
            self._executor.spin_once(timeout_sec=0.2)

    def _update_streams(self) -> None:
        now = time.monotonic()
        if now < self._next_check:
            return
        self._next_check = now + 1.0
        for s in SIDES:
            wanted = (not self._lazy) or self._jpeg_pub[s].get_subscription_count() > 0
            if wanted:
                self._idle_since[s] = None
                if s not in self._jpeg_sub:
                    self._jpeg_sub[s] = self._in_node.create_subscription(
                        CompressedImage, f"/g1/hand/{s}/camera/image_raw/compressed", self._jpeg_cb(s), qos_profile_sensor_data
                    )
            elif s in self._jpeg_sub:
                if self._idle_since[s] is None:
                    self._idle_since[s] = now
                elif now - self._idle_since[s] > 5.0:
                    self._in_node.destroy_subscription(self._jpeg_sub.pop(s))
                    self._idle_since[s] = None

    def _jpeg_cb(self, side: str):
        def cb(msg: CompressedImage) -> None:
            now = time.monotonic()
            if now - self._last[side] < self._period:
                return
            self._last[side] = now
            self._jpeg_pub[side].publish(msg)

        return cb

    def _avail_cb(self, side: str):
        def cb(msg: Bool) -> None:
            self._avail_pub[side].publish(msg)

        return cb

    def destroy_node(self) -> None:
        self._executor.shutdown()
        self._in_node.destroy_node()
        rclpy.shutdown(context=self._in_ctx)
        super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = HandCameraRelay()
    try:
        node.spin_input()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
