# 03 — Palm cameras

Each Inspire hand has a small colour camera that looks at its own palm. They
are **USB (UVC) devices plugged into the Jetson's USB hub**, not network
cameras (the hands' Ethernet only carries Modbus).

## Devices (on the Jetson, 192.168.123.164)

| | Right hand | Left hand |
|---|---|---|
| USB path | `1-2.2.1` | `1-2.2.2` |
| Video node | `/dev/video6` (+ `/dev/video7`, metadata) | `/dev/video8` (+ `/dev/video9`, metadata) |
| **Stable name** | `/dev/v4l/by-path/platform-3610000.xhci-usb-0:2.2.1:1.0-video-index0` | `/dev/v4l/by-path/platform-3610000.xhci-usb-0:2.2.2:1.0-video-index0` |

* UVC "Abham Image" (HHWei, USB id `1c45:6200`), **640×480 @ 30 fps, YUYV**, colour.
* `/dev/v4l/by-id/*Abham*` is useless: both cameras report the **same serial
  (`HHW001`)**, so one symlink overwrites the other.
* `/dev/videoN` numbers can change after a reboot or replug — they **did**: after
  two Jetson reboots on 2026-11-04 the right camera moved from `/dev/video6` to
  `/dev/video8` and the left one the other way round. The **by-path
  names follow the hub port** and stayed correct. The left/right mapping is therefore valid only
  while the cables stay in the same hub ports — swapping them swaps the hands.
* OpenCV 4.2.0 on the Jetson opens them with `cv2.VideoCapture(path_or_index, cv2.CAP_V4L2)`;
  the `by-path` strings work (verified), opening takes 0.5-1 s, and the frame
  comes back BGR (a blue cable in view shows up blue after `BGR2RGB`).
* **Real frame rate ~14 fps** per camera at 640x480, although the driver
  reports 30: measured 13.7-14.2 fps in YUYV and 12.5-12.8 fps in MJPG, one
  camera open at a time. Looks camera-limited (not investigated; the UVC
  controls such as exposure were deliberately left untouched).
* With the fingers closed, the cameras see past the fingers to the floor.

## How right/left was determined

Cover-the-lens test with audio cues (the person at the robot cannot read the
laptop screen): the user covered the lens of the **right** hand's camera four
times; the mean difference of 16×12 thumbnails against the baseline was ~21×
the noise level on hub port 2.2.1 (then `/dev/video6`) and ~1.2× on port 2.2.2 (then `/dev/video8`). Mean *brightness*
turned out to be a bad metric (covering the lens barely changes it).
Script: `tools/cam_cover_test.py`.

## They drop out for ~1-2 s when the head camera starts streaming

Measured 2026-11-05 with root (`dmesg -T`): the two palm cameras (`1-2.2.1` and
`1-2.2.2`, both behind the 10-port USB 2.0 hub `1a86:809f`) disconnect **together**
and re-enumerate 1-2 s later each time a stream of the D435i is started, 8-10 s
after the start, 7 times out of 7 (the video hub, `gst-launch-1.0` test runs, and
also with the D435i on a USB 2.0 port). While the D435i keeps streaming, no further
drop (13 min and counting). They come back with the same `/dev/video6-9` numbers
and the same `by-path` names, with `speed=480`, `power/control=auto`, and they
deliver frames again (8 frames from each right after a drop). Likely a supply dip
on the shared USB 5 V when the D435i starts streaming (not traced). So any node that
reads them must reopen the device when it vanishes, especially right after the head
video starts or restarts.

**Not reproduced with the ROS driver, 2026-11-06 14:50.** 10 start/stop cycles of `realsense_ros2.sh` (5 with the USB power
settings of docs/06 applied, 5 without; 0-2 hardware resets of the D435i per cycle, counted in `dmesg` by timestamp over
28 s after each start): **no disconnect of the camera hub, of the cameras or of anything behind the 10-port hub, ever.** The
7 of 7 above were with other readers of the D435i (the app's video hub, `gst-launch-1.0`). So with the current setup the
D435i start does not make the camera box drop, and moving the box to another USB-C port for power is not needed for that. If
the drops come back (a node reconnecting every few seconds), `dmesg -T | grep 'USB disconnect'` shows them.

### USB layout as the Jetson sees it (2026-11-06)

Everything external sits behind one Realtek RTS5411 4-port hub, which has a USB 2.0 side (`1-2`, bus 1, 480 Mbit/s) and a USB 3.0
side (`2-2`, bus 2, 5 Gbit/s): its port 2 -> the 10-port USB 2.0 hub `1a86:809f` (`1-2.2`) with the two palm cameras on its ports
1 and 2, plus UART/SPI/I2C/JTAG, a dual serial, a card reader and a USB 10/100 LAN (`eth0`, link down; `eth1`, the robot LAN
`192.168.123.164`, is PCIe); port 3 (USB 3.0 side) -> the D435i (`2-2.3`); port 4 -> the Wi-Fi radio of the AP (internal); port 1 -> nothing
enumerates. How the four USB-C connectors on the robot map to these ports (the user: one powers the box of the hands' arrays,
one goes to the box of the two palm cameras, one to the head camera, one free) was not checked: unplug a cable and see which
`1-2.x` / `2-2.x` entry disappears.

## As ROS 2 images on the laptop (2026-11-05)

`tools/hand_cameras_ros2.sh install|start|stop|status|log` runs the workspace's `g1_hand_camera/hand_camera_node`
(Python package copied to `~/g1_hand_cameras/py` on the Jetson, no colcon, no root) with the Jetson's ROS 2 Foxy,
Python 3.8, OpenCV 4.2. Same reasoning as the RealSense driver (docs/06): in the robot's DDS domain 0 Foxy's DDS
crashes, so the node uses **domain 42** with Fast DDS on 192.168.123.164 and the laptop's
`g1_unitree_bridge/hand_camera_relay` (ter_G1U6 repo) copies the streams to domain 0.

* Only the **JPEG** stream is published (`/g1/hand/{left,right}/camera/image_raw/compressed`, 10 Hz requested,
  quality 80): a raw 640x480 rgb8 frame is 920 kB and does not fit the laptop's 416 kB UDP receive buffer.
* Measured at the laptop in domain 42: right 8.3 Hz, left 8.9 Hz (the cameras do ~14 fps). Both cameras opened at once
  without trouble; the node logs `left/right palm camera available`.
* `available` (latched `Bool`) is relayed too. Both USB devices are in `power/control=auto` and show `suspended`
  when idle; they wake up when opened.
* Seen live in the 2D view (ter_G1U6 `hand_2d`): both cameras look at the palm side with the finger bases at the
  top of the picture, i.e. the same way up as the flat-hand diagram.
* Stop it with `hand_cameras_ros2.sh stop`.
* **Autostart (installed 2026-11-06):** `hand_cameras_ros2.sh autostart on|off|status` puts one `@reboot` line in the
  `unitree` user crontab (same mechanism as the RealSense one, docs/06): wait for 192.168.123.164, **60 s** (so that
  the RealSense autostart, whose reset makes the USB hub drop the palm cameras, is done), `start`, one retry; log
  `~/g1_hand_cameras/boot.log`. Tested without a reboot under `env -i`: both cameras available. Needs the package
  copied once by `install`.

## Not done yet

* No intrinsic calibration; no camera-to-palm extrinsics.
* No ROS publisher in this repo (the application workspace has one; `hand_cameras_ros2.sh` runs it on the Jetson).
