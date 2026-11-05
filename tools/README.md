# tools/

Small scripts used to map the robot. Python 3, standard library + `numpy`
(+ `rclpy` and the `unitree_api` messages for `slam_call.py`). Run them with
`python3 -I` where it says so, so the current directory cannot shadow modules.

| Script | What it does | Writes to the robot? |
|---|---|---|
| `modbus_probe.py HOST` | Reads the main registers of one hand (Modbus function 0x03 only) | no |
| `modbus_capture.py SECONDS PREFIX` | Reads bytes 0…5999 of **both** hands repeatedly and saves `PREFIX_210.npz` / `PREFIX_211.npz` (`x[:, k]` = word at byte `2k`) | no |
| `tactile_probe.py HOST [SECONDS] [--map]` | Live read-only view of the 17 tactile regions (sum / max / active cells, optional heat map) | no |
| `cam_cover_test.py LEAD CYCLES` | Left/right test for the palm cameras: streams thumbnails from the Jetson over SSH while the laptop beeps; needs `G1_PW`, `play` (sox) and `askpass.sh` | no (opens the two video nodes) |
| `lidar_listen.py [SECONDS]` | Subscribes to `/utlidar/*_livox_mid360`, prints rates, cloud layout and valid-point stats, saves one cloud to `lidar_cloud.npz` | no |
| `slam_call.py API_ID 'JSON'` | Sends **one** `unitree_api` request to `/api/slam_operate`; `--dry` only checks that someone listens | **yes** (one SLAM request) |
| `g1_service.py list\|on NAME\|off NAME` | Lists / switches the robot's services (the app's Service Status page, e.g. `lidar_driver`) with ONE `unitree_api` request to `/api/robot_state/request` (api 1003 list, 1001 switch); `--dry` only checks that someone listens; `off` refuses safety services | **yes** (`on`/`off`; `list` is a query) |
| `g1_up.sh` | After every robot boot: waits for the service manager, switches `lidar_driver` on if it is off and starts the RealSense driver if it is not running (`G1_PW` = Jetson password, `G1_CAMERA=false` for LiDAR only); idempotent | **yes** (one service switch, the Jetson driver) |
| `tactile_session.py HOST SECS LABEL...` | Voice-guided press test: a 2 s baseline, then for every LABEL it speaks the cue (Italian, `spd-say`), beeps (`play`/sox), records `SECS` and prints the regions that rose above the baseline, with the peak cell | no |
| `slide_scan.py HOST SECS LABEL ...` | Same for a slow slide of a finger along the hand: prints, every 0.5 s, the active region, rows and columns | no |
| `open_hand.py HOST` | Opens **one** hand slowly: `SPEED_SET` 300, `ANGLE_SET` 1000 (thumb rotation kept), aborts and holds if `ERROR` ≠ 0 or `FORCE_ACT` > 700, restores `SPEED_SET` 1000 | **yes** (`SPEED_SET`, `ANGLE_SET`) |
| `start_app_video.sh start\|stop\|status` | **Runs on the Jetson.** Makes the phone app show the head camera after a boot: stops the stock `video_hub_pc4`, runs a patched copy in `/tmp` that streams on the right NIC (see docs/01) | **yes** (stops/starts a Unitree service; nothing persistent) |
| `realsense_ros2.sh install\|start\|stop\|status\|log` (+ `realsense_ros2_remote.sh`, which it sends to the Jetson) | Installs and runs the RealSense D435i ROS 2 driver on the Jetson (own DDS domain 42), see docs/06. `start` stops the app video (`video_hub_pc4`) because the camera cannot be shared | **yes** (files in `~/g1_realsense`, stops a Unitree service; nothing system-wide) |
| `askpass.sh` | `SSH_ASKPASS` helper: prints `$G1_PW` so `ssh` can log in non-interactively | — |

Examples:

```bash
python3 -I tools/modbus_probe.py 192.168.123.211            # right hand
python3 -I tools/modbus_capture.py 20 /tmp/hands            # both hands, 20 s
G1_PW=... python3 tools/cam_cover_test.py 15 4              # which camera is which
```

The password is never stored in the repo: pass it through the environment.
