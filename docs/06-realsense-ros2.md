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
  (`ros2_ws/`).
* **It cannot share the camera with Unitree's `videohub_pc4`.** When the driver first opened the D435i,
  `videohub_pc4_patched` (the app video, see 01) died with `An error occurred!` within seconds.
  `start` therefore stops `video_hub_pc4` (and the patched copy) first: **the phone app has no head
  video while the driver runs.** After `realsense_ros2.sh stop`, `start_app_video.sh start` brings it back.

## Verified (2026-11-05)

Driver on the Jetson: depth 424x240 and colour 640x360, requested at 15 fps. Measured at the laptop
**after** the `ros2_ws` relay (`g1_unitree_bridge/realsense_relay`, which also rate-limits), over the cable:

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
* **Supervisor** (`realsense_ros2_remote.sh supervise`, started by `boot` after the checks, also by hand): since 2026-11-09 it
  looks every 2 s for the driver's own "camera lost" line and restarts at once (see "Dropouts" below); as a fallback every ~30 s
  it runs `check` and restarts the driver after 2 failed checks in a row (the "running but dead" state of 2026-11-06: node
  alive, 0 frames). Tested by SIGSTOP of the driver: restarted by itself in ~2 min. It also acted by itself at 18:49, and at 21:04:56
  Jetson time during a Wi-Fi test (frames stopped, back ~1.5 min later); causes not found. `stop` touches
  `~/g1_realsense/user_stop` so the supervisor does not bring back a driver you stopped on purpose; `start` removes it;
  `autostart off` kills the supervisor. Look for `supervise:` lines in `~/g1_realsense/boot.log`.
  **What the dead state looks like** (log kept by the supervisor as `~/g1_realsense/run.before-restart.<mmdd-HHMMSS>.log`,
  restart of 2026-11-06 22:21:09): the driver process stays alive, the camera works for about 47 s after the start, then at
  22:18:53 libusb returns `Device or resource busy` on `control_transfer` and `usb_request_queue`, `uvc streamer watchdog
  triggered` appears for the colour and depth endpoints, `reset_endpoint` fails, and the same error repeats about 3700 times
  until the supervisor restarts the driver. The kernel logged nothing at that moment. The camera is on USB 3.2 (5000M), so
  it is not bandwidth. What is installed: librealsense 2.50.0 (its release notes recommend D400 firmware 5.13.0.50) and
  camera firmware 05.15.01.55, plus `hwmon command 0x80 failed (HW not ready)` at start. The mismatch is a hypothesis, not a
  verified cause; changing the firmware or the library has not been tried. Restarts counted on 2026-11-06 (Jetson time,
  from `boot.log`): 18:49, 19:01 (my SIGSTOP test), 21:04, 21:09, 21:25, 21:35, 21:49 (during my stop/start tests), 22:17,
  22:21, also with the runtime USB fix active.
* **The kernel messages `Disable of device-initiated U1/U2 failed` + `reset SuperSpeed Gen 1 USB device` are not a sign of
  failure**: they appear at every deliberate hardware reset of the camera (`initial_reset:=true`), also in the clean boot of
  2026-11-06 20:39 (streaming at the first attempt, 115 frames in 8 s). An earlier version of this page took them for the
  cause of the dead state. A runtime test on 2026-11-06 (root on the Jetson: `power/control=on` for the camera, its hub and
  port, `usb3_lpm_permit=0` on the port) worked and the camera streamed, but the driver's reset re-enumerates the device and
  `power/control` goes back to `auto`, so a lasting version would need a udev rule. Not installed: no evidence yet that
  USB power management is the cause.
* **Stream encoder** (`tools/realsense_encoder_node.py`, Foxy, node `realsense_encoder`): `/camera/color/image_raw` ->
  `/camera/color/image_raw/compressed` (JPEG, 10 Hz, q80, ~21 kB) and `/camera/depth/image_rect_raw` ->
  `/camera/depth/image_rect_raw/compressedDepth` (lossless PNG, 5 Hz, ~46 kB). Each only encodes while somebody subscribes.
  `realsense_ros2.sh start|autostart on` copy it with the remote script (`push_files`); `start` / the supervisor start it,
  `stop` stops it. `realsense_ros2.sh stream [name=value ...]` shows / changes its rates, JPEG quality and PNG level live.
  It is what makes the head camera affordable over the Wi-Fi (07, "Compression"). It is a single script next to the
  remote script, not a colcon package: nothing to build on the Jetson.
* `running()` also checks `/proc/<pid>/cmdline`: the pid file survives a reboot and its pid can belong to an
  unrelated process then, which would make `start` say "already running" and never start the driver.
* Every boot loses the phone-app head video (`videohub_pc4` is stopped, see above).
* The LiDAR service has no equivalent on the Jetson: it is switched from the laptop (`tools/g1_up.sh`).
* From the laptop without a terminal for ssh: `G1_PW`, `SSH_ASKPASS=tools/askpass.sh`, `SSH_ASKPASS_REQUIRE=force` and
  `G1_JETSON_SSH="ssh -o StrictHostKeyChecking=accept-new -o PreferredAuthentications=password -o PubkeyAuthentication=no unitree@192.168.123.164"`.

## Dropouts: what is known, how the driver recovers (closed 2026-11-09)

The camera stops delivering frames every 10-20 minutes. **The cause is not fixed; the recovery is fast and automatic.**

* **Stack.** The Jetson now runs librealsense 2.54.2 + realsense-ros 4.54.1 (built in `~/g1_realsense_254`, no root; the
  firmware 5.15.1.55 is accepted and `HW not ready` is gone); `realsense_ros2.sh stack 254|old` chooses it (marker file
  `~/g1_realsense/stack`, applied at the next driver start), `old` is the 2.50.0 + 4.0.4 pair. **Neither stack avoids the
  dropouts**: old stack 0.5-10 min between deaths (once 55); new stack ~20 min on 2026-11-06, and on 2026-11-09 a start
  that was dead from the first second, then 13.5 min of streaming (18:01:15-18:14:38 Jetson time).
* **What happens, with times (2026-11-09 18:14:37-38).** The kernel logs `usb 2-2.3: reset SuperSpeed Gen 1 USB device
  number N using tegra-xusb` (preceded, as at every reset, by `Disable of device-initiated U1/U2 failed`), uvcvideo
  re-binds the camera, and in the same second the driver (a libusb client) gets `Device or resource busy` on every
  endpoint and logs `uvc streamer watchdog triggered`; the kernel adds ~400 `usbfs: process ... did not claim interface 0`.
  The camera never runtime-suspended, the link is USB 3.2, nothing is logged before the reset. A reset the host starts
  by itself with no error before it points at the physical side (cable, connector, supply glitch while `persist` is on),
  **not tested**: the one test left is another USB-C cable or port for the head camera.
* **Recovery.** `dead_signature()` finds `uvc streamer watchdog triggered` after the line `RealSense Node Is Up` in
  `run.log` (4 of 4 saved logs of runs that died: yes, always >= 2 lines, the first in the second of the kernel reset;
  never before `Node Is Up`, where the deliberate `initial_reset` happens; none in a healthy run). The supervisor checks
  it every 2 s, saves the dead run's log and restarts the driver **without waiting for SIGINT** (the `ros2 launch` process
  ignores SIGINT because it is started with `&` from a non-interactive shell, `SigIgn` checked on the Jetson, so the 12 s wait
  ended in SIGKILL every time and not only for a stuck driver as this page used to say; the camera node below it does get
  the SIGINT in a normal `stop`). `start` waits for `Node Is Up` (~13 s) instead of a fixed 20 s, and `_restart` sleeps 2 s instead of 5.
  Expected gap per death: about **20 s** (it was ~85 s: two failed frame checks 30 s apart).
* **A second way to fail, seen twice on 2026-11-09:** after the node's own `Resetting device...` it never finds the camera
  again although the kernel has already re-enumerated it (no `Node Is Up`, 0 frames; the first start after the boot
  at 18:00, and the restart of my simulated-death test at 18:21). A new start works. The supervisor now tries again at once
  when `Node Is Up` does not appear within the start's 24 s (up to 3 attempts, ~50 s worst case with one failed attempt).
  What was tested: the detection of an injected "camera lost" line (restart began 1 s later); in that test the first
  restart hung as described and the old 30 s frame check recovered it 142 s after the injection; the retry code was added
  after that. **Real deaths afterwards (same day, Jetson time):** reset 18:33:51 -> supervisor 18:33:53 -> streaming again
  18:34:11 (**20 s**); reset 18:36:04 -> supervisor 18:36:13 -> first restart never came up (`attempt 1, the node did not
  come up`, 18:36:44) -> second attempt up 18:37:02 (**58 s** in all). So the retry path works in real life. Gaps between
  deaths that day: 13.5, 10.6, 1.9 min, then more than 8 min.
* **For whoever consumes the head camera** (continuous use: navigation and manipulation): plan for a gap of **~20-60 s every
  10-20 min** until the cause is found. Judge the picture by its header stamp (age), not by "the topic exists"; a policy or
  a map builder should pause or fall back when the last frame is older than a few seconds, and carry on by itself when the
  stream comes back (the topic names and the encoder node survive a restart; the encoder is stopped and started with the
  driver). Do not use the head camera as the only obstacle sensor while walking.
* **What others report (searched 2026-11-09): same symptoms, no confirmed fix.** Orin NX + D435, JetPack 5.1.3, disconnects
  after 10-15 h with `-32` / `-71`, `Disable of device-initiated U1/U2 failed`, fallback to USB 2.0, a second UVC device on
  the same hub also hit; closed without an answer
  ([NVIDIA forum](https://forums.developer.nvidia.com/t/realsense-d435-and-additional-uvc-device-usb-disconnections-on-jetson-orin-nx-three-scenarios-with-tegra-xusb-errors-emi-and-potential-hub-resets/342718)).
  Orin NX 16 GB, JetPack R36.4.7, the same `uvc streamer watchdog triggered`: NVIDIA's advice was the firmware the library
  recommends, no IR streams / pointcloud / align, 15 fps everywhere, and a larger USBFS memory buffer
  ([Isaac ROS thread](https://forums.developer.nvidia.com/t/intel-realsense-d435-uvc-streamer-watchdog-errors-and-missing-intrinsics-with-isaac-ros-nvblox/365520)).
  Xavier NX, same watchdog line: solved there by lowering both streams to 15 fps and `initial_reset`
  ([realsense-ros #2149](https://github.com/IntelRealSense/realsense-ros/issues/2149)). A search hit on an AGX Xavier
  thread says a PCIe-to-USB card solved it (only the snippet was read). **Checked here:** firmware 5.15.1.55 is the family
  the installed library recommends (`D4XX_RECOMMENDED_FIRMWARE_VERSION "5.15.1.0"` in 2.54.2), so no mismatch; we already
  run 15 fps, 424x240 depth and 640x360 colour with `initial_reset`; `over_current_count` is 0 on every port of both hubs;
  `usbfs_memory_mb` is 16 (not changed: the failures start with a kernel reset, not with a buffer error).
* **IR projector off: does not stop it (2026-11-09, 21:38-22:20 Jetson time).** The 2.54 launch file has no emitter
  argument, so `depth_module.emitter_enabled: 0` goes in a yaml given as `config_file:=` (the node reported 0 afterwards).
  To make that survive the supervisor's restarts, `start` adds the one line in `~/g1_realsense/extra_args`, if the file
  exists, to the launch arguments (delete the file to go back to the defaults). Result: **2 deaths in 41 min** (21:56:07 and
  22:12:45, 4 kernel resets), with the usual signature, so the projector's supply peak is not the trigger. The rate
  (~1 per 20 min) lies inside the spread seen that day (2 deaths in the first 8 min after the 19:28 boot, none in the 23 min
  after the 21:15 boot, projector on both times), so a smaller effect is neither shown nor excluded. In both deaths the first
  restart did not come up (`attempt 1, the node did not come up`; at 21:56 attempts 1 and 2 both failed), like the 19:29:57 one,
  so the second failure mode above is the usual case after a boot, not the exception; the exact length of those gaps was
  not recorded (streaming again within 13 min and 7 min of the deaths, that is all the log says).
* The palm cameras are no longer affected by those restarts in the normal case: they are opened only on demand (03), so
  a closed palm camera has nothing to lose when the USB hub drops them while the head camera restarts.

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
  the `ros2_ws` viz (`d435_link` → optical frames, rotation only).
