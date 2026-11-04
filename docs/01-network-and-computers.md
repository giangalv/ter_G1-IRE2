# 01 — Network and computers

Measured on 2026-11-04 with a laptop wired to the robot's Ethernet.

## Devices on the LAN (`192.168.123.0/24`)

| IP | What | Reachable services seen |
|---|---|---|
| `192.168.123.99` | the dev laptop (interface `enp59s0`, /24) | — |
| `192.168.123.161` | PC1, Unitree motion-control computer (assumed from the usual G1 layout) | answers ping; **SSH closed** |
| `192.168.123.164` | PC2, **Jetson Orin NX** (hostname `ubuntu`) | ping, **SSH (22)** |
| `192.168.123.120` | **Livox MID-360** LiDAR, head (MAC `ec:72:f7:66:84:f7`) | see [04](04-lidar-mid360.md) |
| `192.168.123.210` | **left** Inspire hand | TCP **6000** open, 502 closed |
| `192.168.123.211` | **right** Inspire hand | TCP **6000** open, 502 closed |

The LAN has no other host; in particular the palm cameras are **not** IP
cameras (they are USB devices of the Jetson, see [03](03-palm-cameras.md)).

Laptop side: `enp59s0` was already configured as `192.168.123.99/24`; no
gateway is needed for the robot LAN.

## The Jetson (PC2, `192.168.123.164`)

* Login: user `unitree` (Unitree's default credentials — ask the lab; not stored here).
* JetPack 5.1.1, Ubuntu 20.04, Python 3.8, OpenCV (cv2) 4.2.0, `gst-launch-1.0`.
  No `ffmpeg`, no `v4l2-ctl`.
* ROS: only **foxy** and **noetic** are installed — no Humble. A Humble stack
  has to run in Docker, and the `unitree` user cannot use docker (permission denied).
* Running services: Unitree `master_service`, `videohub_pc4`. No Inspire service.
* `unitree` is in the `video` group (camera nodes are readable).
* USB devices seen: RealSense D435i (`/dev/video0-5`), the two palm cameras
  (`/dev/video6-9`), WCH serial chips (`ttyACM0-2`, purpose unknown), a WCH
  "USB 10/100 LAN" (`eth1`, no carrier), card reader, Realtek WiFi 6 — **no LiDAR on USB**.
* Livox SDK2 libraries and headers are installed (`/usr/local/lib/liblivox_lidar_sdk_*`)
  but no lidar process runs.

## Talking DDS from the laptop

The Unitree stack uses ROS 2 on **domain 0** with **CycloneDDS**. From a
laptop that is wired to the robot LAN:

```bash
unset ROS_LOCALHOST_ONLY            # a shell profile with ROS_LOCALHOST_ONLY=1 hides the robot
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces>
  <NetworkInterface name="enp59s0" multicast="true"/></Interfaces></General></Domain></CycloneDDS>'
ros2 topic list --no-daemon          # the daemon caches the first environment it saw
```

Pitfalls met on the way:

* `ROS_LOCALHOST_ONLY=1` and a non-zero `ROS_DOMAIN_ID` (e.g. 99) in `~/.bashrc`
  make the robot invisible. Cyclone then picks `lo` and shows only local topics.
* Without the Unitree message packages (`unitree_hg`, `unitree_go`, `unitree_api`)
  built on the laptop, `ros2 topic echo` of Unitree topics fails. The message
  definitions are in the official `unitree_ros2` GitHub repository.
* The ROS 2 daemon caches the discovery configuration of the first command;
  use `--no-daemon` or `ros2 daemon stop` after changing the environment.
