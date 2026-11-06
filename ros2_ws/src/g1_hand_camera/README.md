# g1_hand_camera

Palm cameras of the Inspire hands as ROS 2 images. Each RH56E2 hand has a small
UVC camera looking at its own palm; both are USB devices on the **Jetson's**
hub, so this package runs on the Jetson tier.

## Node: `hand_camera_node`

One capture thread per hand (OpenCV `read` blocks); the ROS side only publishes.

| Topic | Type | QoS | Notes |
|---|---|---|---|
| `/g1/hand/{left,right}/camera/image_raw` | `sensor_msgs/Image` (`rgb8`) | sensor | only while subscribed |
| `/g1/hand/{left,right}/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` (`jpeg`) | sensor | only while subscribed |
| `/g1/hand/{left,right}/camera/camera_info` | `sensor_msgs/CameraInfo` | sensor | `K = 0` (uncalibrated, REP-104) unless a matching calibration file is set |
| `/g1/hand/{left,right}/camera/available` | `std_msgs/Bool` | latched | False while the camera cannot be opened / read |

`frame_id`s: `left_hand_camera`, `right_hand_camera`. No TF is published: the
camera-to-palm extrinsics are unknown.

Hot-plug: when a camera cannot be opened or a read fails, `available` goes
False and the open is retried with exponential backoff (2 s → 30 s).

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `device_left` / `device_right` | `/dev/v4l/by-path/…usb-0:2.2.2…` / `…2.2.1…` | V4L2 device. The mapping follows the **hub port**; swapping cables swaps hands |
| `width`, `height`, `capture_fps` | 640, 480, 30 | requested capture mode |
| `publish_rate_hz` | 30 | decimation of the published stream (0 = every frame) |
| `publish_raw`, `publish_compressed` | true, true | enable each representation |
| `jpeg_quality` | 85 | 1–100 |
| `calibration_left` / `calibration_right` | `""` | ROS `camera_calibration` YAML; ignored (with a warning) if its resolution differs from the stream |
| `frame_id_left` / `frame_id_right` | `{side}_hand_camera` | |
| `reconnect_initial_s` / `reconnect_max_s` | 2 / 30 | reconnect backoff |
| `mock` | false | synthetic frames, no camera and no OpenCV needed |

If `mock` is false and OpenCV is missing, startup fails (CONVENTIONS.md mock rules).

Measured on the lab robot (Jetson, Python 3.8, OpenCV 4.2.0): the by-path names
open with `CAP_V4L2`, frames come out RGB-correct and the real cameras deliver
only **~14 fps** at 640x480 although 30 is requested (same with YUYV and MJPG,
one camera open at a time), so `publish_rate_hz: 30` just means "everything
the camera produces". `rclpy` (Humble) needs `array.array('B')` for `uint8[]`
fields: assigning `bytes` costs ~80 ms per 640x480 frame, hence
`camera_core.to_u8_array`.

## Run

```bash
ros2 run g1_hand_camera hand_camera_node --ros-args -p mock:=true     # no hardware
ros2 launch g1_bringup g1_full_system.launch.py tier:=jetson          # real cameras (Jetson)
rqt_image_view /g1/hand/right/camera/image_raw
```

On the lab robot the node also runs directly on the Jetson's Foxy without colcon:
`tools/hand_cameras_ros2.sh install|start` in the base repo copies this package to `~/g1_hand_cameras/py` and starts
it in DDS domain 42 with `publish_raw:=false` (JPEG only, 10 Hz); `g1_unitree_bridge/hand_camera_relay` brings it to
domain 0 (measured 8-9 Hz per camera). Otherwise `ros2` runs in the Humble container (the Jetson itself only
has foxy/noetic). The container needs the video devices: see the
`jetson_orin` service in `docker-compose.yml`.

## Layout

* `g1_hand_camera/camera_core.py` — rclpy-free: `OpenCvFrameSource`,
  `MockFrameSource`, `ReconnectBackoff`, `RateLimiter`, calibration loading,
  JPEG encoding. Python 3.8 compatible (the Jetson system Python).
* `g1_hand_camera/hand_camera_node.py` — the thin ROS shell.
* `test/test_camera_core.py` — unit tests (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q`).

Device identification and left/right mapping of the physical cameras:
see the `ter_G1-IRE2` robot reference (`docs/03-palm-cameras.md`).
