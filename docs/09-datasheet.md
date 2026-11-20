# 09 — Datasheet of our G1 (U6)

One page with the characteristics of the lab's robot, checked against a retailer's product page. Measured on **2026-11-20** with the
robot standing on its support stand, laptop on the cable. Nothing was written to the robot: every value below was read.

* **Store page** = <https://robottech-store.com/unitree-g1-edu-flagship-d-u6-robotic-humanoid/> (*Unitree G1 Edu Flagship D-U6, 41 DOF*), fetched 2026-11-20.
  It is a reseller, not Unitree's documentation, and it contradicts itself in places (noted below).
* **Status**: ✓ confirmed on our robot · ≈ consistent but derived or from the model (not measured on the hardware) · ✗ differs from the page · ? not verified.
* Serial numbers, MAC addresses and passwords are deliberately not in this page. `tools/g1_inventory.sh` regenerates the Jetson part (it masks them).

## At a glance

| | Our robot | Status |
|---|---|---|
| Variant | G1 U6: 29 body joints + two Inspire hands (6 actuators each) = **41 actuated DOF** | ✓ |
| Main computer (PC1) | page: "8-core high-performance CPU"; SSH is closed, nothing readable | ? |
| AI computer (PC2) | **Jetson Orin NX 16 GB**, 8 cores, MAXN power mode, 100 TOPS (module spec) | ✓ |
| Hands | labels `RH56E2-2L-T1` / `RH56E2-2R-T1` (page: RH56DFTP) | ✗ name |
| LiDAR | Livox MID-360, head | ✓ |
| Depth camera | Intel RealSense D435i, head | ✓ |

## Body and actuation

| | Store page | Our robot | Status |
|---|---|---|---|
| DOF | 41: legs 6 + 6, waist 3, arms 7 + 7, hands 6 + 6 | `/lowstate` has **29** motor slots with data out of 35; the URDF has 29 body + 24 finger joints, 12 of them mimic = 12 actuated; each hand answers with 6 actuator values | ✓ |
| Legs (per leg) | 6 | hip pitch, roll, yaw, knee, ankle pitch, roll | ✓ |
| Waist | 3 | yaw, roll, pitch | ✓ |
| Arms (per arm) | 7 | shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw | ✓ |
| Peak knee torque | 120 N·m | URDF effort limit **139 N·m** (knee and hip roll); the URDF is Unitree's model, not a bench measurement | ✗ |
| Encoders / motors | dual encoder per joint, internal-rotor PMSM | not readable from outside | ? |
| Robot type code | — | `/lowstate` `mode_machine = 5` (Unitree's SDK echoes it in commands; its meaning is not documented in our sources) | — |
| State rate | — | `/lowstate` ≈ 1.03 kHz seen from the laptop (5172 messages in 5 s) | ✓ |
| Walking speed 2 m/s, arm payload ~3 kg | page | no locomotion or load test was made | ? |

Joint limits from the URDF `g1_29dof_rev_1_0_with_inspire_hand_FTP` (unitree_ros commit `4005a1b`), the model the twin uses:

| Joints | Torque limit | Speed limit |
|---|---|---|
| hip pitch, hip yaw, waist yaw | 88 N·m | 32 rad/s |
| hip roll, knee | 139 N·m | 20 rad/s |
| ankle pitch/roll, waist roll/pitch | 35 N·m | 30 rad/s |
| shoulder pitch/roll/yaw, elbow, wrist roll | 25 N·m | 37 rad/s |
| wrist pitch, wrist yaw | 5 N·m | 22 rad/s |

Position ranges (rad) are in the URDF (`robots/g1_description/` of `unitreerobotics/unitree_ros`); the twin copy is `assets/g1/` of the project repository.
At the check: motor temperatures 25–36 °C (two sensors per motor).

## Hands

| | Store page | Our robot | Status |
|---|---|---|---|
| Model | Inspire **RH56DFTP** | labels read on the hands on 2026-11-04: **`RH56E2-2R-T1`** (right) and **`RH56E2-2L-T1`** (left); `E2` = second generation, `T1` = resistive tactile variant ([02](02-inspire-rh56e2-hands.md)) | ✗ |
| Actuators | 6 per hand ("+12 joints" on the same page) | 6 (`ANGLE_ACT` returns 6 values); 12 joints in the URDF because 6 of them are mimic | ✓ |
| Tactile | "17 tactile points" | **17 regions, 1062 cells**, 16-bit counts 0…4095, registers 3000…5123; the layout is the one of the RH56DFTP manual | ✓ (17 = regions) |
| Interface | — | Modbus TCP, port 6000, unit 1, on the LAN: left `.210`, right `.211`; byte-addressed registers | — |
| State at the check | — | both answered, `ERROR` = 0, both hands in a closed fist | ✓ |

The twin's hand model is the **FTP** URDF, while the labels say **E2**: whether the finger and palm geometry differ has not been checked.

## Computers

| | Store page | Our robot | Status |
|---|---|---|---|
| PC1 (motion control, `.161`) | "8-core high-performance CPU" | answers ping, SSH closed; model and cores unknown | ? |
| PC2 (`.164`) | NVIDIA Jetson Orin NX **16 GB, 100 TOPS** | see below | ✓ |

PC2, from `tools/g1_inventory.sh`:

| | |
|---|---|
| Board string | `NVIDIA Orin NX Developer Kit` (the generic device-tree name of the module) |
| OS | Ubuntu 20.04.6, kernel 5.10.104-tegra, aarch64, **L4T R35.3.1 = JetPack 5.1.1** |
| CPU | **8 cores**, ARMv8, up to 1984 MHz (8 cores is the 16 GB module; the 8 GB one has 6) |
| Memory | **15.0 GiB** (MemTotal 15,757,812 kB), swap 7.5 GiB (zram) |
| Power mode | `nvpmodel` reports **MAXN (id 0)** (it could not read one sysfs file without root); `/etc/nvpmodel.conf` names mode 2 (15 W) as the default, so MAXN was set afterwards |
| Compute | **100 TOPS** is NVIDIA's peak INT8 (sparse) figure for the 16 GB module in MAXN: a datasheet number, not a benchmark of our workloads |
| Storage | NVMe SSD 2 TB (1.9 TiB), 41 GB used |
| Temperature | 34–37 °C idle |
| Software | ROS 2 Foxy + ROS 1 Noetic (no Humble), Python 3.8.10, Docker 24.0.7; Unitree modules `master_service`, `video_hub_pc4` |
| Network | `eth1` 1 Gb/s on the robot LAN (the second Ethernet name flips between boots, see [01](01-network-and-computers.md)); `wlan0` access point |

## Sensors

| | Store page | Our robot | Status |
|---|---|---|---|
| 3D LiDAR | Livox MID-360 | `/utlidar/cloud_livox_mid360` 10 Hz (~20 k points per frame, about half are no-returns) and `/utlidar/imu_livox_mid360` 200 Hz, frame `livox_frame`; mounted upside down in the head ([04](04-lidar-mid360.md)). The `lidar_driver` service is off after every boot (`tools/g1_up.sh`) | ✓ |
| Depth camera | Intel RealSense D435i | USB `8086:0b3a` on the Jetson's USB 3 hub, tilted 47.6° down in the URDF ([06](06-realsense-ros2.md)); the stream drops every 10–20 min and recovers in ~20 s | ✓ |
| Body IMU | not stated | quaternion in `/lowstate` | ✓ |
| Palm cameras | not on the page | two UVC cameras (`1c45:6200`), 640×480, about 14 fps ([03](03-palm-cameras.md)) | ✓ (our addition) |
| Microphones, speaker | 4-microphone array, 5 W stereo speaker | no audio device on the Jetson besides its HDMI and internal APE cards; `/audio_msg`, `/api/audiohub`, `/api/voice`, `/api/vui` exist on DDS, so audio is handled elsewhere (PC1?) | ? |

## Connectivity

| | Store page | Our robot | Status |
|---|---|---|---|
| Wi-Fi | Wi-Fi 6 | Realtek **RTL8852BU** USB adapter on the Jetson (driver `rtl8852bu`, a Wi-Fi 6 chip family); it runs the robot's access point on **2.4 GHz, channel 6, 20 MHz**, with a new name at every boot ([07](07-wifi.md)); it died once on its own (2026-11-13) | ≈ |
| Bluetooth | 5.2 | controller reports HCI/LMP version 0xB = Bluetooth 5.2 (same Realtek chip, driver `rtk_btusb`) | ✓ |
| Ethernet | not stated | Gigabit on the robot LAN `192.168.123.0/24`: PC1 `.161`, Jetson `.164`, LiDAR `.120`, hands `.210` / `.211` | ✓ |

## Software seen on the robot's DDS (domain 0)

Services on `/api/*`: `sport`, `arm`, `motion_switcher`, `robot_state`, `robot_type_service`, `slam_operate`, `audiohub`, `voice`, `vui`, `videohub`, `gpt`, `vla`,
`action_store`, `dex3_msg_controller`, `bashrunner`, `config`, `basic_softlimit`, `basic_taumax` and others; low-level topics `/lowstate`, `/lowcmd`, `/arm_sdk`,
`/hand_sdk`, `/dex3/*`. Details in [05](05-unitree-dds.md) and [08](08-motion-and-remote.md).

## What the page says that we did not confirm, and what the page does not say

* ✗ Hand model name (DFTP on the page, E2 on the hands) and ✗ peak knee torque (120 on the page, 139 in the URDF).
* ? PC1's CPU, the microphone array and speaker, top speed and payload.
* The page's "Wi-Fi 6" is only inferred from the chip family; no 802.11ax link was observed (the access point runs 2.4 GHz, 20 MHz).
* The page does not give the IMU, the Ethernet ports or the torque of any joint but the knee.

## Regenerating

```bash
G1_PW=<Jetson password> tools/g1_inventory.sh > inventory.txt      # Jetson, read-only, masks MACs   (cable; G1_JETSON_IP=10.42.0.1 on the Wi-Fi)
python3 tools/modbus_probe.py 192.168.123.210 ; python3 tools/modbus_probe.py 192.168.123.211            # hands, read-only
```

The `/lowstate` count, rate and temperatures came from a short subscriber (read-only, DDS environment of docs/01) and the URDF numbers from parsing the file named above.
