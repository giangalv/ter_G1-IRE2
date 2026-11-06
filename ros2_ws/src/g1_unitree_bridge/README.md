# g1_unitree_bridge — Unitree native state and video → workspace topics (read-only)

The G1 publishes its state on the DDS topics `/lowstate` (~1 kHz) and `/lf/lowstate` (~20 Hz) as
`unitree_hg/msg/LowState` (domain 0). `lowstate_bridge` subscribes to ONE of them and republishes

| Out | Type | Content |
|---|---|---|
| `/g1/joint_states` | `sensor_msgs/JointState` | BODY-41 order (CONVENTIONS.md): position `q`, velocity `dq`, effort `tau_est`; slots 29-40 = 0 |
| `/g1/imu/data` | `sensor_msgs/Imu` | quaternion (Unitree w,x,y,z → ROS x,y,z,w), gyro, accelerometer |

It never publishes towards the robot. Parameters: `lowstate_topic` (default `/lowstate`),
`max_rate_hz` (default 500, 0 = every message), `imu_frame`.

Needs the message types `unitree_hg` (+ `unitree_go`, `unitree_api` for `ros2 topic list -t`):
`bash src/g1_description/fetch_assets.sh` downloads them into `third_party/` and colcon builds them.

Unitree's motor order (legs 0-11, waist 12-14, left arm 15-21, right arm 22-28) is permuted into
BODY-41 by `bridge_core.BODY41_TO_UNITREE`; `mode_machine` 5 (`g1_29dof_rev_1_0`) was verified on
the robot, the first positions checked against `/lowstate` by hand.

```bash
ros2 run g1_unitree_bridge lowstate_bridge --ros-args -p lowstate_topic:=/lf/lowstate
```

## `head_video`: D435i colour from Unitree's multicast video

Joins 230.1.1.1:1720 (RTP/H.264 from `videohub_pc4` on the Jetson) with GStreamer through OpenCV and
publishes `/g1/camera/color/image` (rgb8, `frame_id` d435_link). Parameters: `iface` (NIC facing the
robot, e.g. `enp59s0`), `scale` (0.5: 1280x720 → 640x360; 1.0 drops most frames because the 2.7 MB
samples overflow the default UDP buffers), `max_rate_hz` (10). Needs the sender running (see
`tools/start_app_video.sh` in the base repo). Colour only; depth and IMU of the D435i are not in this stream.

```bash
ros2 run g1_unitree_bridge head_video --ros-args -p iface:=enp59s0
```

## `realsense_relay`: D435i depth, colour and cloud from the RealSense driver

The RealSense ROS 2 driver (realsense-ros 4.0.4) runs on the Jetson in DDS domain 42 because Foxy's DDS
crashes in the robot's domain 0 (`tools/realsense_ros2.sh` and docs/06 in the base repo). The relay opens a
second rclpy context in domain 42 (`rs_domain`) and only receives there; in the robot's domain it publishes

| Out | Content |
|---|---|
| `/g1/camera/depth/image`, `.../camera_info` | 32FC1 metres (NaN = no measurement), frame `d435_depth_optical_frame`, `image_rate_hz` 10 |
| `/g1/camera/depth/points` | XYZ float32 cloud, every `stride`-th pixel (2), 0.1 m .. `max_range` (6 m), `points_rate_hz` 5 |
| `/g1/camera/color/image`, `.../camera_info` | rgb8 640x360, frame `d435_color_optical_frame`, `color_rate_hz` 10 |

The optical frames are tied to the URDF's `d435_link` by static transforms in `g1_viz.launch.py`
(rotation only, no depth/colour offset). The depth maths (`depth_core.py`) is tested without ROS.

```bash
ros2 run g1_unitree_bridge realsense_relay --ros-args -p rs_domain:=42
```

## `hand_camera_relay`: palm cameras from the Jetson's DDS domain

Same two-context pattern as `realsense_relay`: it receives `/g1/hand/{left,right}/camera/image_raw/compressed`
(JPEG) and the latched `.../camera/available` in domain 42 (`g1_hand_camera/hand_camera_node` running on the Jetson with
Foxy, started by `tools/hand_cameras_ros2.sh` of the base repo) and publishes the same names in the robot's domain.
Parameters: `rs_domain` (42), `max_rate_hz` (10).

```bash
ros2 run g1_unitree_bridge hand_camera_relay
```
