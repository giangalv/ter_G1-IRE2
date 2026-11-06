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

## Freezes after a robot reboot (2026-11-05)

After rebooting the robot, the first driver run streamed ~70 s and then the log filled with
`uvc streamer watchdog triggered on endpoint 130/132`, `reset_endpoint ... Device or resource busy`
(9600 lines in a few minutes) while the node stayed "running": the image in RViz froze. A plain restart gave depth
but **no colour** (`Hardware Notification: Depth stream start failure` although depth arrived). Restarting with
`initial_reset:=true` (hardware reset of the camera at node start, no root needed) gave both streams at ~10.5 Hz
raw and no error for the 3 minutes watched; `start` now sets it by default and waits 20 s. The root cause was not
found (`dmesg` is not readable without sudo and was not escalated); it is not `videohub_pc4` (not running).
If it freezes again: `realsense_ros2.sh stop && realsense_ros2.sh start`, and look at `log`.
The relay at the laptop depends on the laptop's load: with another heavy job running (load average 31, swap full)
it dropped to ~2.5 Hz while the raw topics were still at ~10 Hz.

## Autostart at boot (installed 2026-11-06)

`tools/realsense_ros2.sh autostart on|off|status` adds / removes one `@reboot` line in the **user** crontab of
`unitree` on the Jetson (it copies `realsense_ros2_remote.sh` to `~/g1_realsense`, waits for 192.168.123.164,
sleeps 20 s, runs `start`, retries once; log `~/g1_realsense/boot.log`; no root). The user asked for it on 2026-11-05
(the permission check refused that first call) and confirmed on 2026-11-06; it is **installed**.

* **A running driver is not a streaming driver.** On 2026-11-06 the first start after a robot boot (the autostart test,
  launched ~8 min after the boot) had `control_transfer returned error` from its first second, ~10 per second for an hour
  (32 000 lines), the process alive and **0 frames** on both streams; `stop` + `start` fixed it at once (15 Hz colour and
  depth). I had only checked "Node Is Up" and wrongly called it working. Now `realsense_ros2_remote.sh check` counts
  depth frames for 8 s (exit 1 if < 20), `boot` uses it and restarts up to 3 times, and `stop` escalates to SIGKILL
  when the driver ignores SIGINT (it did: "did not stop"). The retry path itself has not been seen in real life: the
  tested case is a clean start under `env -i` (cron's environment): attempt 1, 121 frames in 8 s. Read
  `~/g1_realsense/boot.log` after the next robot boot.
* **Supervisor** (`realsense_ros2_remote.sh supervise`, started by `boot` after the checks, also by hand): every 30 s it runs
  `check` and restarts the driver after 2 failed checks in a row (a USB reset like the one of 2026-11-06 at 18:22 Jetson time, "Disable of
  device-initiated U1/U2 failed", leaves the node alive and 0 frames). Tested by SIGSTOP of the driver: restarted by itself in
  ~2 min. `stop` touches `~/g1_realsense/user_stop` so the supervisor does not bring back a driver you stopped on purpose;
  `start` removes it; `autostart off` kills the supervisor. Root cause of the reset not verified (suspects: USB3 U1/U2
  power management, D435i `power/control=auto`); a fix would be a root change on the Jetson.
* **JPEG colour node** (`tools/realsense_jpeg_node.py`, Foxy, `realsense_color_jpeg`): `/camera/color/image_raw` ->
  `/camera/color/image_raw/compressed` at 10 Hz, quality 80, ~21 kB per frame (57 frames in 8 s measured), encodes only while
  somebody subscribes. `realsense_ros2.sh start|autostart on` copy it with the remote script (`push_files`);
  `start` / the supervisor start it, `stop` stops it. It is what makes the head colour affordable over the Wi-Fi (07).
* `running()` also checks `/proc/<pid>/cmdline`: the pid file survives a reboot and its pid can belong to an
  unrelated process then, which would make `start` say "already running" and never start the driver.
* Every boot loses the phone-app head video (`videohub_pc4` is stopped, see above).
* The LiDAR service has no equivalent on the Jetson: it is switched from the laptop (`tools/g1_up.sh`).
* From the laptop without a terminal for ssh: `G1_PW`, `SSH_ASKPASS=tools/askpass.sh`, `SSH_ASKPASS_REQUIRE=force` and
  `G1_JETSON_SSH="ssh -o StrictHostKeyChecking=accept-new -o PreferredAuthentications=password -o PubkeyAuthentication=no unitree@192.168.123.164"`.

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
