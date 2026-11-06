# g1_viz — RViz2 visualization from any PC

Passive visualization of the G1 U6 over the robot LAN (WiFi or Ethernet):
LiDAR cloud, camera streams, DynaMem obstacle map, odometry, TF and — with a
URDF — the animated robot model.

```bash
ros2 launch g1_viz g1_viz.launch.py                          # auto-find URDF
ros2 launch g1_viz g1_viz.launch.py urdf:=/path/to/g1.urdf
ros2 launch g1_viz g1_viz.launch.py use_rviz:=false          # relay only
```

## Nodes

### `joint_state_relay`

The VLA stack publishes `/g1/joint_states` as a flat BODY-41 vector
(CONVENTIONS.md ordering), which `robot_state_publisher` cannot consume
directly. This node republishes it as a URDF-named `sensor_msgs/JointState`
on `/joint_states` and (optionally) broadcasts the `odom -> base_link` TF from
`/g1/odom` so the model moves in the world frame.

| Parameter | Default | Meaning |
|---|---|---|
| `publish_rate_hz` | 50.0 | Output rate of `/joint_states` |
| `publish_odom_tf` | true | Broadcast `odom -> base_link` from `/g1/odom` |
| `odom_frame` / `base_frame` | odom / base_link (the launch passes `pelvis`, the root of Unitree's URDFs) | TF frame names |
| `body_joint_names` | 29 g1_description names | URDF names in BODY-41 index order; empty string drops a slot |
| `hand_joint_names_left/right` | 6 Inspire joints of the FTP URDF | Names in HAND order (little, ring, middle, index, thumb_flex, thumb_rot); empty = fingers not animated |
| `hand_joint_closed_rad` | [1.4381 ×4, 0.5864, 1.1641] | Joint angle at normalised position 1.0 (closed) |

## Robot model (URDF)

Unitree's model is not stored in git. One command downloads it (pinned commits) into
`third_party/` and fills the `g1_description` package; the launch then finds it by itself:

```bash
bash src/g1_description/fetch_assets.sh
colcon build --symlink-install --packages-up-to g1_viz g1_description unitree_hg unitree_go unitree_api
```

The default model is `g1_29dof_rev_1_0_with_inspire_hand_FTP` (G1 29 DoF, `mode_machine` 5, Inspire
hands with 17 force-sensor links per hand, the same count as the RH56E2-T1 regions). Fingers: the
relay drives the six actuated joints of each hand from `/g1/hand/{left,right}/joint_states`
(normalised 0 open .. 1 closed, scaled by `hand_joint_closed_rad`); with no hand message they stay
open. Without a URDF everything still works except the robot mesh.

## Real robot over the cable (read-only)

With the robot on, the PC on the robot's Ethernet (192.168.123.x) and **no VLA stack running**:

```bash
make viz_cable                 # = bash src/g1_viz/scripts/viz_real.sh cable
G1_NIC=eth0 make viz_cable     # other PC: name of its Ethernet NIC (default enp59s0)
```

What it does: `g1_unitree_bridge/lowstate_bridge` subscribes the robot's `/lf/lowstate` (20 Hz;
`G1_LOWSTATE_TOPIC=/lowstate` gives ~1 kHz at the price of a CPU core) and publishes
`/g1/joint_states` (BODY-41) + `/g1/imu/data`; the relay maps them onto the URDF names;
`robot_state_publisher` + RViz2 draw the model. With `fixed_base:=true` the pelvis is held 0.79 m
above the floor (no odometry on the stand). Nothing is ever published towards the robot.

Pitfalls found on the real robot:
* The robot's DDS is **domain 0**; the script sets `ROS_DOMAIN_ID=0` and `ROS_LOCALHOST_ONLY=0`
  (a shell with `ROS_LOCALHOST_ONLY=1` sees no remote topic at all).
* `config/cyclonedds_cable.xml` does not request a 10 MB socket buffer: Cyclone aborts when
  `net.core.rmem_max` is lower (416 kB by default) and RViz does not need it.
* Installed config files are copies: rebuild `g1_viz` after editing `config/`.
* `ros2 topic hz/echo` need `--qos-profile sensor_data` for these best-effort topics.
* Several launches in a row: stop one with SIGINT (Ctrl-C). `pkill` on the launch leaves its nodes
  running, and a second `inspire_hand_bridge_node` opens a second Modbus connection to each hand.

## Hands: tactile cells on the model

`make viz_cable` also starts `inspire_hand_bridge_node` (**read-only**: commands are dropped, only
registers are read over Modbus TCP, 192.168.123.210 left / .211 right) and `tactile_markers`.
Every one of the 17 regions per hand is drawn on its force-sensor link of the URDF (`pad`, `nail`,
`tip`, thumb `middle`, `palm`): one cube per cell above `threshold` (15 counts) coloured blue → red up
to `full_scale` (400 counts), plus a translucent sphere on the plate (bigger and redder with the
strongest cell) so a touch is visible from afar. The first 30 frames are taken as zero: **do not
touch the hands while it starts**; `ros2 service call /g1/viz/tactile/rezero std_srvs/srv/Empty` redoes it.

* Zoomed views: `G1_VIEW=hand_right make viz_cable` / `hand_left` (default `full`).
* Values are raw counts, not newtons (no force calibration exists).
* Orientation of the grid inside a plate is not documented by the manufacturer. `tactile_core.py`
  lays rows along the long side of the plate for the finger plates; the palm has columns along the
  short side with column 0 on the finger side. Override with the `layout_json` parameter, e.g.
  `{"index_pad": {"flip_rows": true}, "palm": {"rows_axis": "short", "cols_axis": "long"}}`.

## 2D view: the hands laid flat, with the palm cameras

`hand_2d` opens one OpenCV window (`G1_HAND_2D=false` skips it) with both hands as seen from the palm side, fingers
up, thumb unfolded to the side (left hand mirrored), every one of the 1062 cells as a small quadrilateral:
grey = no contact, blue → red above `threshold` (15 counts over the baseline, red at `full_scale` 400), a white
outline around every region that is touched, the strongest region named at the top right. Below each hand: its palm
camera (`camera:=above` puts it on top, `none` hides it).

* The positions come from the same plates as the 3D view: the URDF is walked at zero joint angles (open hand), every
  cell centre is projected onto the palm plane; the thumb plates stand on edge there, so they are laid along the
  thumb axis, seen from the side the sensors face (fingertip side). Orientation overrides (`layout_json`) change both
  views. Drawn from the model, not measured: if a touch lights a different spot than the one you press, change the
  layout (the flip flags of that region, e.g. `{"left_palm": {"flip_rows": true}}`) and tell me.
* Keys in the window: `z` takes the zero again (do not touch), `q` / Esc quits. `ros2 service call
  /g1/viz/hand_2d/rezero std_srvs/srv/Empty` does the same. The window is plain OpenCV, not an RViz image: the
  composed picture is ~1 MB, which the laptop's 416 kB UDP buffer would not carry.
* Palm cameras: the nodes run on the Jetson (`tools/hand_cameras_ros2.sh install`, then `start`, in the base repo,
  DDS domain 42 like the RealSense) and `hand_camera_relay` (`G1_HAND_CAMERAS=false` skips it) copies the JPEGs into
  the robot's domain as `/g1/hand/{left,right}/camera/image_raw/compressed`. Without them the panel says so.
  The cameras drop for 1-2 s when a D435i stream starts.
* Standalone: `ros2 run g1_viz hand_2d --ros-args -p snapshot_path:=/tmp/hands.png -p show:=false` writes the picture
  once a second without a window.

## Head camera and LiDAR

* **D435i colour + depth + cloud (RealSense driver, default)**: the camera's ROS 2 driver runs on the
  Jetson in its own DDS domain 42 (Foxy's DDS crashes in the robot's domain 0). Install it once and start
  it with the tools of the base repo: `tools/realsense_ros2.sh install`, then `tools/realsense_ros2.sh start`
  (docs/06-realsense-ros2.md). `g1_unitree_bridge/realsense_relay` (started by `viz_real.sh`, `G1_REALSENSE=false`
  skips it) copies domain 42 into the robot's domain as `/g1/camera/color/image` (rgb8 640x360),
  `/g1/camera/depth/image` (32FC1, metres) and `/g1/camera/depth/points` (XYZ cloud in the optical frame),
  plus the two `camera_info`. Seen over the cable: colour ~8 Hz, depth ~6.6 Hz, cloud ~3.4 Hz (~19k points).
  While the driver runs the phone app has no head video (the camera cannot be shared). No D435i IMU
  (Motion Module hardware error), cloud not coloured, depth not aligned to colour.
* **D435i colour only (alternative)**: `head_video` (g1_unitree_bridge) joins Unitree's multicast H.264 video
  (230.1.1.1:1720, sent by `videohub_pc4` on the Jetson) and publishes `/g1/camera/color/image`
  (rgb8, 640x360 at `scale` 0.5, ~7-10 Hz). It needs the sender running: after every robot boot run
  `ssh unitree@192.168.123.164 'bash -s -- start' < tools/start_app_video.sh` from the base repo
  (docs/01-network-and-computers.md). Switched on with `G1_HEAD_CAMERA=true` (default false; do not run it
  together with the RealSense driver, both publish the same topic). Starting that stream makes the palm
  cameras drop for ~1-2 s.
* **After every robot boot** the LiDAR service and the RealSense driver are off again. From the base repo:
  `G1_PW=<Jetson password> tools/g1_up.sh` switches `lidar_driver` on (the call the app makes, `g1_service.py`)
  and starts the driver, leaving what already runs alone. Or let the viewer do it first:
  `G1_PW=... G1_BRINGUP=~/Documents/GitHub/ter_G1-IRE2/tools/g1_up.sh make viz_cable`. Nothing becomes permanent on
  the robot; `G1_CAMERA=false` keeps the phone-app video.
* **LiDAR**: RViz shows `/utlidar/cloud_livox_mid360` (frame `livox_frame`, tied to the URDF's
  `mid360_link` by an identity transform). The robot publishes it (10 Hz) only while the **`lidar_driver`
  service is on**: phone app, *Service Status*; if it is Close, the display stays empty (docs/04 in the base repo).
  The orientation of that transform was checked with the LiDAR IMU's gravity vector (within ~0.5 degrees);
  the position is the URDF's, not verified.
* GStreamer here has no `h264parse` (gst-plugins-bad); the pipeline works without it.

## Bandwidth over WiFi

RViz subscriptions are best-effort, so a weak link degrades gracefully
(dropped frames, never a stall). To keep the radio budget small, throttle the
heavy streams **onboard** and point RViz at the throttled topics:

```bash
# on the Jetson
ros2 run topic_tools throttle messages /g1/lidar/points 2 /g1/lidar/points_viz
ros2 run topic_tools throttle messages /g1/camera/color/image 5 /g1/camera/color/image_viz
```

### Over the robot's own Wi-Fi access point (measured 2026-11-06)

`bash src/g1_viz/scripts/viz_real.sh wifi` after `G1_PW=<Jetson password> tools/g1_wifi.sh up` (base repo ter_G1-IRE2, which
also explains the setup: a discovery relay on the Jetson and a second laptop address 192.168.123.98, docs/07-wifi.md).
The Cyclone config is `config/cyclonedds_wifi.xml` (interface address `$G1_WIFI_ADDR`, multicast for the discovery packet only).
The link carries ~60-100 Mbit/s, less than everything offered, so `G1_WIFI_PROFILE` picks what is on, by task:

| `G1_WIFI_PROFILE` | shown (RViz config) | measured (nodes only) |
|---|---|---|
| `nav` | LiDAR cloud + head camera (colour, depth cloud); no hands, no palm cameras, no 2D view (`g1_real_nav.rviz`) | LiDAR 9.8 Hz, colour 3.5 Hz, depth cloud 4.0 Hz, joint states 50 Hz |
| `manip` (default) | hands tactile + 2D view, palm cameras, head camera; LiDAR off (`g1_real_manip.rviz`) | tactile 6 / 15 Hz (with PNG depth), palm cameras ~6 Hz, colour 4.7 Hz, depth cloud 3.1 Hz |
| `full` | everything (`g1_real.rviz`) | tactile 2-3 Hz, colour 3.2 Hz, depth cloud 1.9 Hz: does not fit |

On the Wi-Fi the head camera travels compressed, made on the Jetson (base repo `tools/realsense_encoder_node.py`): colour as
JPEG (~1.7 Mbit/s instead of ~66) and depth as lossless 16-bit PNG at 5 Hz (~1.8 Mbit/s instead of ~23); the relay decodes
both and publishes the same `/g1/camera/*` topics as before. Choose with `G1_RS_COLOR=jpeg|raw` and `G1_RS_DEPTH=png|raw`
(default jpeg + png on `wifi`, raw on `cable`); how much the Jetson sends (rates, JPEG quality, PNG level) is changed live
with `tools/realsense_ros2.sh stream ...` in the base repo (docs/07-wifi.md, "Compression"). The relays are also **lazy**:
`realsense_relay` and `hand_camera_relay` pull a stream from the Jetson only while
something subscribes to their output and release it 5 s after the last subscriber (`lazy` parameter, default true). In RViz
that means ticking a display starts its traffic and unticking it stops it, whatever the profile started. The raw LiDAR
cloud and the tactile topics are not relayed: RViz / the hand bridge take them directly, so an unticked LiDAR display costs nothing.

The hand bridge's Modbus timeout is 0.15 s there (`hand_tcp_timeout_s`, 0.02 on the cable); some timeouts remain while
the link is loaded (7 in ~60 s in `manip`, none with the hands off). `nav`/`full` need the `lidar_driver` service on
(`tools/g1_up.sh` with `G1_CYCLONEDDS_URI=file://$HOME/.cache/g1_wifi_cyclonedds.xml`).
Not verified: RViz itself over the Wi-Fi (the tests ran the nodes without windows), a walking robot, distance.
