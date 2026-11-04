"""Subscribe (read-only) to the Livox MID-360 DDS topics for N seconds.

Usage: python3 tools/lidar_listen.py [SECONDS]      (default 15)

Needs the DDS environment of docs/01 (domain 0, CycloneDDS on the robot NIC).
Prints the rate of /utlidar/imu_livox_mid360 and /utlidar/cloud_livox_mid360,
the layout of one cloud and its valid-point statistics, and saves that cloud
to ./lidar_cloud.npz (``pts`` = N x 6: x y z intensity ring time, ``names``).
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
rclpy.init()
n = Node("lidar_listen")
st = {"imu": [], "cloud": []}
clouds = []
qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
n.create_subscription(Imu, "/utlidar/imu_livox_mid360", lambda m: st["imu"].append(time.monotonic()), qos)
def on_cloud(m):
    st["cloud"].append(time.monotonic())
    if len(clouds) < 3:
        clouds.append(m)
n.create_subscription(PointCloud2, "/utlidar/cloud_livox_mid360", on_cloud, qos)
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.1)
for k, v in st.items():
    if len(v) > 2:
        print(f"{k}: {len(v)} msgs in {DUR:.0f} s, ~{(len(v) - 1) / (v[-1] - v[0]):.1f} Hz")
    else:
        print(f"{k}: {len(v)} msgs in {DUR:.0f} s")
if clouds:
    m = clouds[-1]
    print("cloud frame_id:", m.header.frame_id, "| width x height:", m.width, "x", m.height, "| point_step:", m.point_step, "| fields:", [(f.name, f.datatype) for f in m.fields], "| is_dense:", m.is_dense)
    arr = pc2.read_points(m, field_names=[f.name for f in m.fields], skip_nans=True)
    a = np.array([list(p) for p in arr], dtype=np.float64) if not hasattr(arr, "dtype") else np.stack([arr[f] for f in arr.dtype.names], axis=1).astype(np.float64)
    names = [f.name for f in m.fields]
    xyz = a[:, :3]
    r = np.linalg.norm(xyz, axis=1)
    valid = r > 0.05  # about half of the points are (0, 0, 0): no return in that direction
    print("points: %d, valid (range > 5 cm): %d" % (a.shape[0], valid.sum()))
    print("valid range min/median/max (m): %.2f / %.2f / %.2f" % (r[valid].min(), np.median(r[valid]), r[valid].max()))
    v = xyz[valid]
    print("x range %.2f..%.2f  y range %.2f..%.2f  z range %.2f..%.2f" % (v[:,0].min(), v[:,0].max(), v[:,1].min(), v[:,1].max(), v[:,2].min(), v[:,2].max()))
    np.savez("lidar_cloud.npz", pts=a, names=np.array(names))
n.destroy_node(); rclpy.shutdown()
