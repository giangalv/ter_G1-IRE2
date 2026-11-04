# 04 — LiDAR (Livox MID-360)

## What it is and where

* **Livox MID-360** in the head, **Ethernet only** (no USB), IP
  `192.168.123.120`, MAC `ec:72:f7:66:84:f7`. Confirmed by the user.
* Its driver does **not** run on the Jetson: the Jetson's `master_service` only
  manages `ota_pipe` and the video hub. The point cloud comes from Unitree's own
  software on the other computer (PC1, `192.168.123.161`, no SSH), which
  republishes it on DDS.

## It streams: read it from DDS (domain 0)

Verified 2026-11-04 with `tools/lidar_listen.py` (read-only subscriber):

| Topic | Type | Rate | Publisher |
|---|---|---|---|
| `/utlidar/cloud_livox_mid360` | `sensor_msgs/PointCloud2` | **10 Hz** | 1 (Unitree, a bare-DDS app), RELIABLE / VOLATILE |
| `/utlidar/imu_livox_mid360` | `sensor_msgs/Imu` | **200 Hz** | 1 (same), RELIABLE / VOLATILE |

Both use `frame_id: livox_frame`. Nobody else was subscribed. No extra
configuration is needed beyond the DDS environment of
[01](01-network-and-computers.md).

Cloud layout: `width = 20063, height = 1`, `point_step = 22`, fields
`x, y, z` (float32, metres), `intensity` (float32, 0-255), `ring`
(uint16, only the values 0-3 were seen), `time` (float32, 1e4 … 1.0e8 observed, looks like nanoseconds inside the 100 ms frame).
About **48 % of the points are exactly (0, 0, 0)**, i.e. no return in that
direction; drop them (range < 5 cm). A typical frame had ~9.7k valid points,
range median 2.3 m, up to 17 m, z from -1.45 to +1.76 m. The top view of one
frame showed the walls and furniture of the room, so it is a real scan.

The IMU's `orientation` is all zeros (not provided) and its angular velocity is
filled in; the linear acceleration was not inspected.

Not known yet: the transform `livox_frame` → robot base (no TF is published for
it), so the cloud is in the sensor frame.

## What I got wrong first

Earlier the same day I concluded the LiDAR was **not streaming**. I had only
looked at `/unitree/slam_mapping/points` and `/unitree/slam_relocation/points`
(PointCloud2, **0 publishers** — they only exist while Unitree's SLAM runs) and
at the Jetson's UDP ports (the Livox pushes to PC1, not to the Jetson). The raw
LiDAR topics are the `/utlidar/*_livox_mid360` ones above.

## SLAM (separate from the raw stream)

With the user's approval, one SLAM-start request was sent with `tools/slam_call.py`:

```
topic    /api/slam_operate/request   (unitree_api/msg/Request, 1 subscriber)
api_id   1801   parameter {"data":{"slam_type":"indoor"}}   # "start mapping"
```

No response arrived and no `/unitree/slam_mapping/*` publisher appeared within
32 s. The api ids come from a third-party mirror of Unitree's documentation and
are **unverified**. Nothing else was sent. This has no bearing on the raw
`/utlidar/*` stream, which works without SLAM.

## Other topics seen

`/utlidar/range_info` (`geometry_msgs/PointStamped`, 0 publishers, 1 subscriber),
`/unitree_slam/high_rate_odometry`, `/state_estimator/*` odometry topics.
