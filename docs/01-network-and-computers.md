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

### Network interfaces and Unitree services on the Jetson

* The robot LAN (`192.168.123.164/24`, NetworkManager profile `unitree1`) is on
  the **onboard PCIe NIC** (driver `r8169`). The interface names are **not
  stable across boots**: on 2026-11-04 the onboard NIC was `eth1` and the
  "USB 10/100 LAN" adapter (QinHeng `1a86:5395`, `cdc_ether`, no carrier) was
  `eth0`; earlier in the day it was the other way round. Select by MAC or
  driver, never by name.
* `master_service` (`/unitree/module/master_service`) starts and monitors the
  Unitree services; its log is `/unitree/var/log/master_service/master_service.LOG`.
  A service that fails to stay up is restarted every ~5 s and, after a
  "protect count" of **30**, is **given up for good** until it is started again
  (`mscli startservice <name>`) or the Jetson reboots with the cause fixed.
* `video_hub_pc4` runs `videohub_pc4 /dev/video4`: it reads the **RealSense D435i
  colour stream** (`/dev/video4`, 1920x1080 YUY2 @ 15 fps via `v4l2src`), encodes
  it with the Jetson hardware encoder (`nvv4l2h264enc`, `nvjpegenc`) and
  publishes it over DDS — presumably what the phone app shows as video
  (not verified). On 2026-11-04 it was dead: it exited right after every start
  since at least 19:26, the master gave up (protect count 30), and the
  RealSense colour node `/dev/video4` only appeared at 19:59 (its USB cable
  had apparently been unplugged until then). A manual 5-frame
  `gst-launch-1.0 v4l2src device=/dev/video4 ! video/x-raw,format=YUY2,width=1920,height=1080,framerate=15/1`
  worked, so the camera itself was fine at that point; the exit cause is unknown.
* `video_hub_pc4_chest` runs on `/dev/video10`, which does not exist on this unit.

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
