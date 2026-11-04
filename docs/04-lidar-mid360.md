# 04 — LiDAR (Livox MID-360)

## What it is and where

* **Livox MID-360** in the head, **Ethernet only** (no USB), IP
  `192.168.123.120`, MAC `ec:72:f7:66:84:f7`. Confirmed by the user.

## State on 2026-11-04: not streaming

* Nothing on the Jetson receives its data: no lidar process runs, nothing
  listens on UDP 56301 / 56401 (Livox SDK2 point/IMU ports) and the UDP
  `NoPorts` counter does not move. Whoever is configured to receive the stream
  is probably PC1 (`.161`, not accessible) or Unitree's own stack.
* The Livox SDK2 libraries are installed on the Jetson, and old logs
  (Nov 2023) show `livox_ros_driver2`, LIO-SAM and a Hesai node once ran from
  `/unitree/module/graph_pid_ws`; that workspace no longer exists.
* The DDS graph advertises `/unitree/slam_mapping/points`,
  `/unitree/slam_relocation/points` (PointCloud2) and `/utlidar/range_info`,
  all with **0 publishers**.

## SLAM start attempt

With the user's approval, one request was sent with `tools/slam_call.py`:

```
topic    /api/slam_operate/request   (unitree_api/msg/Request, 1 subscriber)
api_id   1801   parameter {"data":{"slam_type":"indoor"}}   # "start mapping"
```

No response arrived (`/api/slam_operate/response` has no publisher) and no
`/unitree/slam_mapping/*` publisher appeared within 32 s. The api ids come
from a third-party mirror of Unitree's documentation and are **unverified**
(the official pages need JavaScript). Nothing else was sent; no "close SLAM"
(1901) was needed.

## Options for next time

1. Use the Unitree phone app to start mapping, then look again at
   `/unitree/slam_*/points` and `/utlidar/*` on DDS domain 0.
2. Re-point the Livox to another host with the Livox SDK2 configuration (needs
   the host IP and ports). **This takes the stream away from whatever
   consumes it now** — only after agreeing on it.
3. Livox SDK2 broadcast discovery on UDP 56000 (needs the SDK2 CRC16/CRC32
   framing, not attempted).
