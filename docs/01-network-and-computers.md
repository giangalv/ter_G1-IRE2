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
  serves it over DDS — presumably what the phone app shows as video (not
  verified). It uses its **own DDS config**,
  `/unitree/module/video_hub_pc4/cyclonedds.xml`, which pins CycloneDDS to the
  interface **`eth0`** by name.
* **Root cause of the dead video service on 2026-11-04 (verified):** the
  interface names flipped, so `eth0` became the USB LAN adapter (no carrier, no
  address) and the robot LAN moved to `eth1`. Run by hand with the stock config,
  the hub prints `eth0: does not match an available interface` /
  `dds_create_participant: Error` and aborts (exit 134) — that is why
  `master_service` saw it die after every start until its protect count (30)
  ran out. Selecting the interface by address instead,
  `<NetworkInterface address="192.168.123.164" .../>`, makes it start normally
  (NVENC initialises and it keeps running). The kernel names the PCIe NIC
  (`r8169`) and the USB adapter (`cdc_ether`, on the Jetson's internal hub) in
  probe order, so the names can flip again at any boot.
* **Temporary fix used (no file changed, lost at reboot):**
  ```bash
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.123.164" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
  setsid nohup /unitree/module/video_hub_pc4/videohub_pc4 /dev/video4 > /tmp/videohub_pc4_manual.log 2>&1 < /dev/null &
  ```
  Permanent options (all need root on the Jetson, none applied): edit the xml to
  select by `address=`, or add a udev/systemd `.link` rule that pins the PCIe
  NIC to `eth0`.
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
