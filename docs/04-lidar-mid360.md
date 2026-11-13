# 04 — LiDAR (Livox MID-360)

## What it is and where

* **Livox MID-360** in the head, **Ethernet only** (no USB), IP
  `192.168.123.120`, MAC `ec:72:f7:66:84:f7`. Confirmed by the user.
* Its driver does **not** run on the Jetson: the Jetson's `master_service` only
  manages `ota_pipe` and the video hub. The point cloud comes from Unitree's own
  software on the other computer (PC1, `192.168.123.161`, no SSH), which
  republishes it on DDS.

## It streams: read it from DDS (domain 0)

Verified 2026-11-04 with `tools/lidar_listen.py` (read-only subscriber):

| Topic | Type | Rate | Publisher |
|---|---|---|---|
| `/utlidar/cloud_livox_mid360` | `sensor_msgs/PointCloud2` | **10 Hz** | 1 (Unitree, a bare-DDS app), RELIABLE / VOLATILE |
| `/utlidar/imu_livox_mid360` | `sensor_msgs/Imu` | **200 Hz** | 1 (same), RELIABLE / VOLATILE |

Both use `frame_id: livox_frame`. Nobody else was subscribed. No extra
configuration is needed beyond the DDS environment of
[01](01-network-and-computers.md).

Cloud layout: `width = 20063, height = 1`, `point_step = 22`, fields
`x, y, z` (float32, metres), `intensity` (float32, 0-255), `ring`
(uint16, only the values 0-3 were seen), `time` (float32, 1e4 … 1.0e8 observed, looks like nanoseconds inside the 100 ms frame).
About **48 % of the points are exactly (0, 0, 0)**, i.e. no return in that
direction; drop them (range < 5 cm). A typical frame had ~9.7k valid points,
range median 2.3 m, up to 17 m, z from -1.45 to +1.76 m. The top view of one
frame showed the walls and furniture of the room, so it is a real scan.

The IMU's `orientation` is all zeros (not provided) and its angular velocity is
filled in; the linear acceleration was not inspected.

The transform `livox_frame` → robot base is not published by the robot; the URDF's `mid360_link` fits the
orientation (see below), the translation is unverified.

## No cloud? The `lidar_driver` service must be on (2026-11-05)

On 2026-11-05 after a reboot **no** `/utlidar/*` topic had a publisher (also the IMU one was gone, `lidar_listen.py`
got 0 messages in 10 s) while the LiDAR (.120), PC1 and the Jetson answered ping. The cause was a **service switched
off**: in the phone app, *Service Status* (colours: blue Functional, red Abnormal, grey Close) listed `lidar_driver`
(and `ota_box`) as Close. The user switched them on in the app and within seconds the topics were back:

| Topic | Rate measured | Note |
|---|---|---|
| `/utlidar/cloud_livox_mid360` | 10.0 Hz | 20064 points, 9542 with range > 5 cm, median 2.2 m, max 16.9 m |
| `/utlidar/imu_livox_mid360` | 200.0 Hz | `linear_acceleration` is in **g** (|a| = 1.004), not m/s^2 |

Check first with `ros2 topic list -v | grep utlidar` (a subscriber without publisher means the driver is off).

**Without the app** (`tools/g1_service.py`, `tools/g1_up.sh`): the app's switch is the `ServiceSwitch` call of the
`robot_state` service (unitree_ros2 `ros2_robot_state_client.h`: request on `/api/robot_state/request`, api 1001 with
`{"name": NAME, "switch": 0|1}`; api 1003 lists the services). `g1_service.py list` returned 30 services with
`status` and `protect`; **status 0 = running, 1 = stopped** (checked: switched off in the app -> `lidar_driver` 1 and no
data; `g1_service.py on lidar_driver` -> response `status 0`, cloud 10 Hz and IMU ~190 Hz again within seconds).
The switch does not survive a robot reboot (it was Close again after one), so `g1_up.sh` repeats it from the laptop.
The list also holds services the app does not show (`auto_test_arm`, `auto_test_low`, `unistore`, `unitree_slam`,
`vui_service`, `video_hub_pc4`, `video_hub_pc4_chest`, `webrtc_*`); `protect=1` on `basic_service`, `robot_state` and
two `webrtc_*`.

**Mounting, verified with the IMU.** At rest the IMU read `a = (-0.043, 0.003, -1.003)` g, i.e. gravity "up" is the
sensor's **-z**: the MID-360 is mounted upside down, tilted a few degrees. The URDF's `mid360_link`
(`rpy = π 0.051 0` on the head) predicts `(-0.051, 0, -0.999)`: the orientation of `livox_frame` = `mid360_link` is
confirmed to ~0.5 degrees. The translation (0.47 m above the pelvis in the URDF) is not verified; the cloud has a
dense horizontal layer 1.2-1.4 m on the sensor's +z side (= below the head), compatible with the floor.

Services listed by the app on that day, all Functional after the switch (versions as shown): ai_sport 8.7.5.212,
audio_player_service, bashrunner 1.1.1.2, basic_service 3.0.0.18, battery_guard 1.0.0.9, chat_go 1.9.5.26,
dex3_service_l, dex3_service_r, emergency_stop 1.0.0.4, g1_arm_example 2.0.2.5, lidar_driver 1.0.0.12,
log_system 2.0.0.11, motion_switcher 1.0.0.1, net_switcher 1.0.2.1, ota_box, ota_pipe, robot_state 3.0.0.8,
robot_type_service 1.0.2.8, state_estimator 2.0.0.25.

## The front mask and the head's own edges (2026-11-13)

The robot ships with a physical mask in front of the LiDAR. Measured on the robot (hanging, cloud 10 Hz, 20 000 points of which
about 40 % have no return and come as `x = y = z = 0`; "valid" = range > 5 cm; azimuth = `atan2(y, x)` in the sensor frame,
0 = straight ahead; 127 clouds per state):

| | mask fitted | mask removed |
|---|---|---|
| valid points per cloud | 9 382 | 12 224 |
| forward wedge `|az| < 40 deg` | 245 | 3 100 |
| every other azimuth | same | same |

The mask closes exactly the free forward wedge; with it removed the LiDAR sees ahead. **Two shadows remain with or without the mask:
the edges of the head**, at azimuth +40...+60 deg (about 86 points per cloud, median range 0.11 m) and -60...-40 deg (about 236 per cloud,
a third of them closer than 0.6 m). They are fixed in the sensor frame, because the sensor is rigid on the head. The arms and shoulders
also return points at 0.1-0.6 m around +-80...100 deg.

Decision (2026-11-13): the viewer (`g1_viz`) shows the cloud as it comes and does **not** filter these points. Whoever builds on this base
for navigation has to cut them: for walking, drop returns closer than about 0.6 m in the sectors +-40...100 deg (simple range/azimuth
crop in the sensor frame); for manipulation, where the arms enter the view, use a self-filter from the URDF and the joint states instead.

Why Unitree fits the mask is not known. LiDAR on or off made no difference to the D435i depth image (valid pixels, frame-to-frame noise),
and the camera made none to the cloud, but **the fitted mask costs the camera its lower field of view**: the depth image is the same in rows
0-336 and falls to 75 %, 30 % and 2 % valid in rows 336-384, 384-432, 432-480 (D435i, same boot, three states mask on / off / on). Data in
the support bundle, not in this repo.

## What I got wrong first

Earlier the same day I concluded the LiDAR was **not streaming**. I had only
looked at `/unitree/slam_mapping/points` and `/unitree/slam_relocation/points`
(PointCloud2, **0 publishers** — they only exist while Unitree's SLAM runs) and
at the Jetson's UDP ports (the Livox pushes to PC1, not to the Jetson). The raw
LiDAR topics are the `/utlidar/*_livox_mid360` ones above.

## SLAM (separate from the raw stream)

With the user's approval, one SLAM-start request was sent with `tools/slam_call.py`:

```
topic    /api/slam_operate/request   (unitree_api/msg/Request, 1 subscriber)
api_id   1801   parameter {"data":{"slam_type":"indoor"}}   # "start mapping"
```

No response arrived and no `/unitree/slam_mapping/*` publisher appeared within
32 s. The api ids come from a third-party mirror of Unitree's documentation and
are **unverified**. Nothing else was sent. This has no bearing on the raw
`/utlidar/*` stream, which works without SLAM.

## Other topics seen

`/utlidar/range_info` (`geometry_msgs/PointStamped`, 0 publishers, 1 subscriber),
`/unitree_slam/high_rate_odometry`, `/state_estimator/*` odometry topics.
