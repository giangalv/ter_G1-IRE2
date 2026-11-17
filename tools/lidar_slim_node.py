#!/usr/bin/env python3
"""Shrink the head LiDAR cloud on the Jetson, before it crosses the robot's Wi-Fi (docs/07-wifi.md).

Runs on the Jetson (ROS 2 Foxy, Python 3.8, numpy 1.17) in DDS domain 0 over the robot LAN. Reads
/utlidar/cloud_livox_mid360 (what the robot's lidar_driver publishes, ~440 kB per cloud at 10 Hz = 3.7 MB/s) and publishes
the same cloud, same fields, same frame, with fewer points on /g1/lidar/slim:

  * the empty points: x = y = z = exactly 0 (never NaN, never a tiny range), 38-39 % of every cloud on this robot. They
    still carry a `time`, a `ring` and sometimes an `intensity`, so they are scan slots in which the driver wrote a
    placeholder instead of a position; WHY (no laser return, or a point the driver rejected) is not known. Downstream they
    would all pile up at the sensor's origin;
  * the robot's own head and shoulders: returns closer than --head-range (0.6 m) at --head-az-min..--head-az-max degrees
    to the side of the sensor (sensor frame, 0 = ahead). Measured 2026-11-17: only ~330 points per cloud (1.6 %), so this
    alone saves almost nothing, but it is the part the user wants gone, and nothing downstream can use it;
  * optionally --stride N: keep every N-th of the remaining points (default 1 = all).

Nothing else changes (the per-point `time`, `ring`, `intensity` stay), the stamp is the robot's own. Started and stopped
by tools/lidar_slim.sh; nothing persists on the robot.
"""
import argparse
import time
from array import array

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

_DTYPE = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}


class LidarSlim(Node):
    def __init__(self, a):
        super().__init__("lidar_slim")
        self.a = a
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
        self.pub = self.create_publisher(PointCloud2, a.out, qos)
        self.create_subscription(PointCloud2, a.inp, self.on_cloud, qos)
        self.dtype = None
        self.n_in = self.n_out = self.b_in = self.b_out = 0
        self.busy = 0.0
        self.t0 = time.time()
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            "lidar_slim up: %s -> %s (min range %.2f m, head sector %g..%g deg < %.2f m, stride %d)"
            % (a.inp, a.out, a.min_range, a.head_az_min, a.head_az_max, a.head_range, a.stride))

    def _dtype_for(self, m):
        return np.dtype({"names": [f.name for f in m.fields], "formats": [_DTYPE[f.datatype] for f in m.fields],
                         "offsets": [f.offset for f in m.fields], "itemsize": m.point_step})

    def on_cloud(self, m):
        t = time.time()
        a = self.a
        dt = self._dtype_for(m)
        pts = np.frombuffer(m.data, dtype=dt)
        x = pts["x"]
        y = pts["y"]
        z = pts["z"]
        r2 = x * x + y * y + z * z
        keep = np.isfinite(r2) & (r2 > a.min_range ** 2)
        near = np.nonzero(keep & (r2 < a.head_range ** 2))[0]
        if near.size:
            az = np.abs(np.degrees(np.arctan2(y[near], x[near])))
            drop = near[(az >= a.head_az_min) & (az <= a.head_az_max)]
            keep[drop] = False
        idx = np.nonzero(keep)[0][:: a.stride]
        out = PointCloud2()
        out.header = m.header
        out.height = 1
        out.width = int(idx.size)
        out.fields = m.fields
        out.is_bigendian = m.is_bigendian
        out.point_step = m.point_step
        out.row_step = m.point_step * out.width
        out.is_dense = True
        out.data = array("B", pts[idx].tobytes())
        self.pub.publish(out)
        self.n_in += 1
        self.n_out += int(idx.size)
        self.b_in += len(m.data)
        self.b_out += int(idx.size) * m.point_step
        self.busy += time.time() - t

    def report(self):
        dt = time.time() - self.t0
        if self.n_in:
            self.get_logger().info(
                "%.1f clouds/s, in %.2f MB/s -> out %.2f MB/s (%.0f %%), %.0f points/cloud out, %.1f ms of CPU per cloud"
                % (self.n_in / dt, self.b_in / dt / 1e6, self.b_out / dt / 1e6, 100.0 * self.b_out / max(self.b_in, 1),
                   self.n_out / self.n_in, 1000.0 * self.busy / self.n_in))
        else:
            self.get_logger().warn("no cloud from the robot yet (is the lidar_driver service on?)")
        self.t0 = time.time()
        self.n_in = self.n_out = self.b_in = self.b_out = 0
        self.busy = 0.0


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--in", dest="inp", default="/utlidar/cloud_livox_mid360")
    p.add_argument("--out", default="/g1/lidar/slim")
    p.add_argument("--min-range", type=float, default=0.05, help="drop points closer than this (the empty points are exactly 0), metres")
    p.add_argument("--head-range", type=float, default=0.6, help="head/shoulder returns are closer than this, metres")
    p.add_argument("--head-az-min", type=float, default=40.0, help="degrees to the side of the sensor, 0 = ahead")
    p.add_argument("--head-az-max", type=float, default=100.0)
    p.add_argument("--stride", type=int, default=1, help="keep every N-th remaining point")
    a = p.parse_args()
    a.stride = max(1, a.stride)
    rclpy.init()
    node = LidarSlim(a)
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
