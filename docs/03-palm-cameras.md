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
* `/dev/videoN` numbers can change after a reboot or replug; the **by-path
  names follow the hub port**. The left/right mapping is therefore valid only
  while the cables stay in the same hub ports — swapping them swaps the hands.
* OpenCV 4.2.0 on the Jetson opens them with `cv2.VideoCapture(path_or_index, cv2.CAP_V4L2)`.
* With the fingers closed, the cameras see past the fingers to the floor.

## How right/left was determined

Cover-the-lens test with audio cues (the person at the robot cannot read the
laptop screen): the user covered the lens of the **right** hand's camera four
times; the mean difference of 16×12 thumbnails against the baseline was ~21×
the noise level on `/dev/video6` and ~1.2× on `/dev/video8`. Mean *brightness*
turned out to be a bad metric (covering the lens barely changes it).
Script: `tools/cam_cover_test.py`.

## Not done yet

* No intrinsic calibration; no camera-to-palm extrinsics.
* No ROS publisher in this repo (the application workspace has one).
