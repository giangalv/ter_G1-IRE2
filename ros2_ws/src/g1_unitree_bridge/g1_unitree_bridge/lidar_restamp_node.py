"""Copy the LiDAR cloud with this PC's clock as its stamp (read-only: it only subscribes to the robot).

Why: the robot's PC1 keeps its own clock (measured 2026-11-13: 56 s behind the laptop's) and stamps the cloud with it,
while every TF of the viewer (joint states, odometry) is stamped with the laptop's clock. RViz then drops each cloud
("earlier than all the data in the transform cache"). The cloud is re-published with the arrival time, so a viewer needs
no clock sync with the robot (the cost is the link delay, a few ms).

    /utlidar/cloud_livox_mid360  ->  /g1/lidar/points   (same frame_id, same data, new stamp)

``lazy`` (default true, for the robot's Wi-Fi: the cloud is ~3.7 MB/s): the robot's cloud is only subscribed to while
somebody subscribes to the output, and dropped a few seconds after the last one is gone.
"""
from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

IN_TOPIC = "/utlidar/cloud_livox_mid360"
OUT_TOPIC = "/g1/lidar/points"
IDLE_DROP_S = 5.0


class LidarRestamp(Node):
    def __init__(self) -> None:
        super().__init__("lidar_restamp")
        self.declare_parameter("in_topic", IN_TOPIC)
        self.declare_parameter("out_topic", OUT_TOPIC)
        self.declare_parameter("lazy", True)
        self._in_topic = str(self.get_parameter("in_topic").value)
        self._out_topic = str(self.get_parameter("out_topic").value)
        self._lazy = bool(self.get_parameter("lazy").value)
        self._pub = self.create_publisher(PointCloud2, self._out_topic, qos_profile_sensor_data)
        self._sub = None
        self._idle_since: Optional[float] = None
        if self._lazy:
            self.create_timer(1.0, self._check)
        else:
            self._subscribe()
        self.get_logger().info(f"lidar_restamp up: {self._in_topic} -> {self._out_topic} (lazy={self._lazy})")

    def _subscribe(self) -> None:
        if self._sub is None:
            self._sub = self.create_subscription(PointCloud2, self._in_topic, self._on_cloud, qos_profile_sensor_data)
            self.get_logger().info("somebody watches the cloud: subscribing to the robot's")

    def _unsubscribe(self) -> None:
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
            self.get_logger().info("nobody watches the cloud: dropped the robot's stream")

    def _check(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.count_subscribers(self._out_topic) > 0:
            self._idle_since = None
            self._subscribe()
        elif self._sub is not None:
            if self._idle_since is None:
                self._idle_since = now
            elif now - self._idle_since > IDLE_DROP_S:
                self._idle_since = None
                self._unsubscribe()

    def _on_cloud(self, msg: PointCloud2) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarRestamp()
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
