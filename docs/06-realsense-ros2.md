# 06 — RealSense D435i with ROS 2

The head camera is an **Intel RealSense D435i** on the Jetson (USB 3.2, serial `254322070410`,
firmware `05.15.01.55`). Unitree only streams its colour to the phone app; to get
**depth** (and colour) as ROS 2 topics the standard RealSense driver runs on the Jetson.

```bash
tools/realsense_ros2.sh install     # once: sources from GitHub (laptop) -> Jetson, build there (~1.5 min)
tools/realsense_ros2.sh start       # depth 424x240@15, colour 640x360@15; add launch args at the end
tools/realsense_ros2.sh status | log | stop
```

Everything lives in `/home/unitree/g1_realsense` on the Jetson (sources, `install/`, `run.log`);
nothing outside the home directory is touched and no `sudo` is used. `rm -rf ~/g1_realsense`
removes it. The ssh target can be changed with `G1_JETSON_SSH="ssh user@host"`.

## How it is put together (and why)

* **Jetson**: JetPack 5.1.1 / Ubuntu 20.04 with ROS 2 **Foxy** (full) and ROS 1 Noetic, **no internet**. The wrapper is **realsense-ros 4.0.4** (commit
  `8abb4657`), the version made for librealsense **2.50.0** — exactly what `ros-noetic-librealsense2`
  has installed under `/opt/ros/noetic` (headers, `.so`, CMake config). It is built with
  `-Drealsense2_DIR=/opt/ros/noetic/lib/aarch64-linux-gnu/cmake/realsense2` and run with that
  directory on `LD_LIBRARY_PATH`. No `pyrealsense2`, no `rs-*` tools on the Jetson.
* **DDS domain 42, Fast DDS, UDP on `192.168.123.164` only.** In the robot's domain 0 the driver dies:
  Foxy's **Fast DDS** prints `bad_alloc caught` repeatedly and is killed; Foxy's **Cyclone DDS 0.7**
  segfaults in `ddsi_plist_init_frommsg` (libddsc, builtin dqueue) and does not even know
  `<Interfaces>` in `CYCLONEDDS_URI`. The cause was not investigated further; both
  crash only when the robot's other participants are around. In domain 42 only the driver and the laptop
  exist, and Fast DDS (Foxy) ↔ Cyclone (Humble) interoperate fine.
* **The laptop** sees the camera with `ROS_DOMAIN_ID=42` (topics `/camera/...`, standard names), or
  — for the rest of the workspace — through `g1_unitree_bridge/realsense_relay`, which copies
  colour and depth into the robot's domain as `/g1/camera/{color,depth}/...` and builds a cloud
  (ter_G1U6 repo).
* **It cannot share the camera with Unitree's `videohub_pc4`.** When the driver first opened the D435i,
  `videohub_pc4_patched` (the app video, see 01) died with `An error occurred!` within seconds.
  `start` therefore stops `video_hub_pc4` (and the patched copy) first: **the phone app has no head
  video while the driver runs.** After `realsense_ros2.sh stop`, `start_app_video.sh start` brings it back.

## Verified (2026-11-05)

Driver on the Jetson: depth 424x240 and colour 640x360, requested at 15 fps. Measured at the laptop
**after** the ter_G1U6 relay (`g1_unitree_bridge/realsense_relay`, which also rate-limits), over the cable:

| Topic in the robot's domain | Content | Rate seen |
|---|---|---|
| `/g1/camera/depth/image` | 32FC1, metres | ~6.6 Hz |
| `/g1/camera/depth/points` | XYZ cloud, ~19k points | ~3.4 Hz |
| `/g1/camera/color/image` | rgb8 640x360 | ~8 Hz |

The rates of the raw `/camera/...` topics in domain 42 were not measured separately. The profiles are small on
purpose: the laptop's UDP receive buffer is only 416 kB (`net.core.rmem_max`, no sudo), and a 1280x720 rgb8
image is 2.7 MB per sample; larger profiles were not tried through DDS.

## Not working / not done

* **IMU: no.** The motion module reports `Hardware Notification: Motion Module failure, Hardware Error`
  on this Jetson (cause not investigated), so `enable_gyro`/`enable_accel` are `false`.
* Harmless noise in the log: `hwmon command 0x80 ... HW not ready` for auto-exposure-limit options,
  `control_transfer returned error ... Resource temporarily unavailable`.
* Depth aligned to colour and a coloured cloud (`align_depth.enable`), exposure/laser settings, `rs2`
  filters: not tried. `pointcloud.enable` of the driver is off: the cloud is built on the laptop
  (a 424x240 cloud is ~1.2 MB and would not survive the 416 kB buffer).
* Starting any D435i stream makes the two palm cameras drop for ~1–2 s (03).
* The depth origin of the optical frame and the 15 mm colour offset are not modelled in the TF used by
  the ter_G1U6 viz (`d435_link` → optical frames, rotation only).
