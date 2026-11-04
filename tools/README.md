# tools/

Small scripts used to map the robot. Python 3, standard library + `numpy`
(+ `rclpy` and the `unitree_api` messages for `slam_call.py`). Run them with
`python3 -I` where it says so, so the current directory cannot shadow modules.

| Script | What it does | Writes to the robot? |
|---|---|---|
| `modbus_probe.py HOST` | Reads the main registers of one hand (Modbus function 0x03 only) | no |
| `modbus_capture.py SECONDS PREFIX` | Reads bytes 0…5999 of **both** hands repeatedly and saves `PREFIX_210.npz` / `PREFIX_211.npz` (`x[:, k]` = word at byte `2k`) | no |
| `cam_cover_test.py LEAD CYCLES` | Left/right test for the palm cameras: streams thumbnails from the Jetson over SSH while the laptop beeps; needs `G1_PW`, `play` (sox) and `askpass.sh` | no (opens the two video nodes) |
| `lidar_listen.py [SECONDS]` | Subscribes to `/utlidar/*_livox_mid360`, prints rates, cloud layout and valid-point stats, saves one cloud to `lidar_cloud.npz` | no |
| `slam_call.py API_ID 'JSON'` | Sends **one** `unitree_api` request to `/api/slam_operate`; `--dry` only checks that someone listens | **yes** (one SLAM request) |
| `askpass.sh` | `SSH_ASKPASS` helper: prints `$G1_PW` so `ssh` can log in non-interactively | — |

Examples:

```bash
python3 -I tools/modbus_probe.py 192.168.123.211            # right hand
python3 -I tools/modbus_capture.py 20 /tmp/hands            # both hands, 20 s
G1_PW=... python3 tools/cam_cover_test.py 15 4              # which camera is which
```

The password is never stored in the repo: pass it through the environment.
