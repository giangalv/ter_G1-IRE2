# ter_G1-IRE2 — the Unitree G1 U6 with Inspire RH56E2 hands, explained

Basic reference for the lab's **Unitree G1 EDU Ultimate** humanoid: which
computers it has, how they are networked, what the **Inspire RH56E2** hands
speak, where the **palm cameras** and the **LiDAR** show up, and how to talk to
all of it from a laptop. Everything here was measured on the real robot
(hanging on its support stand).

## Contents

| Page | What it covers |
|---|---|
| [docs/01-network-and-computers.md](docs/01-network-and-computers.md) | Devices on the `192.168.123.0/24` LAN, the Jetson, how to connect a laptop, DDS environment |
| [docs/02-inspire-rh56e2-hands.md](docs/02-inspire-rh56e2-hands.md) | Hand model, Modbus TCP on port 6000, **byte-addressed** registers, register map, left/right, **tactile arrays (17 regions, 1062 cells)** |
| [docs/03-palm-cameras.md](docs/03-palm-cameras.md) | The two palm-facing USB cameras: devices, stable names, left/right mapping |
| [docs/04-lidar-mid360.md](docs/04-lidar-mid360.md) | Livox MID-360 in the head: address, **the DDS topics that carry its point cloud (10 Hz) and IMU (200 Hz)** |
| [docs/05-unitree-dds.md](docs/05-unitree-dds.md) | Native Unitree topics seen over DDS (arms, hands, SLAM API) |
| [docs/06-realsense-ros2.md](docs/06-realsense-ros2.md) | D435i depth + colour as ROS 2 topics: driver built on the Jetson, why it needs its own DDS domain, why the app video stops |
| [docs/07-wifi.md](docs/07-wifi.md) | The robot's own access point (random name at every boot), what a Wi-Fi client reaches, how DDS domain 0 works over it (discovery relay on the Jetson, `tools/g1_wifi.sh`), measured rates per viz profile. |
| [tools/](tools/README.md) | Small scripts used to find all of the above |

## The robot at a glance

```
 laptop (192.168.123.99) ──ethernet──┬─ 192.168.123.161  PC1  Unitree motion control (no SSH)
                                     ├─ 192.168.123.164  PC2  Jetson Orin NX (SSH, user "unitree")
                                     │        └─ USB: RealSense D435i (head), 2 palm cameras, ...
                                     ├─ 192.168.123.120  Livox MID-360 LiDAR (head)
                                     ├─ 192.168.123.210  Inspire RH56E2-2L-T1  (LEFT hand,  Modbus TCP :6000)
                                     └─ 192.168.123.211  Inspire RH56E2-2R-T1  (RIGHT hand, Modbus TCP :6000)
```

| Thing | Value |
|---|---|
| Hands | Inspire Robots `RH56E2-2R-T1` / `RH56E2-2L-T1` (6 actuators each, `T1` = resistive tactile variant) |
| Hand protocol | **Modbus TCP**, port **6000**, unit id **1** |
| Hand IPs | **right = .211, left = .210** |
| Palm cameras | 2 × UVC "Abham Image", 640×480 @ 30 fps, on the **Jetson's USB hub** |
| LiDAR | Livox MID-360, Ethernet, `192.168.123.120`; cloud 10 Hz + IMU 200 Hz on DDS `/utlidar/*_livox_mid360` |
| Head camera | Intel RealSense D435i on the Jetson |
| Robot middleware | Unitree stack on ROS 2 DDS **domain 0** (CycloneDDS) |
| Head camera / app video | D435i → Jetson service `video_hub_pc4`; the app shows it only after `tools/start_app_video.sh start` on each boot (see [01](docs/01-network-and-computers.md#phone-app-video-after-every-boot)) |
| Head camera depth / ROS 2 | `tools/realsense_ros2.sh install` once, then `start`: RealSense driver (ROS 2 Foxy) on the Jetson in DDS domain 42; stops the app video (see [06](docs/06-realsense-ros2.md)) |

## Open questions

* Tactile (`T1`): the layout (17 regions, 1062 cells) is decoded and the regions are **located on both hands** (fully on the right, a quick check on the left gave the same layout) (`tip` = extreme end, `nail` = fingerprint side, `pad` = part nearest the hand, `palm`, plus `thumb_middle` in the middle of the thumb; nothing on the back, on the second phalanx or on the lined grip). Still open: the force per count (no calibration).
* The **write path** works for `SPEED_SET` and `ANGLE_SET` on both hands (each opened once, see [docs/02](docs/02-inspire-rh56e2-hands.md)); `FORCE_SET` has never been written.
* The `livox_frame` → robot-base transform of the LiDAR: the orientation matches the URDF's `mid360_link` (checked with the LiDAR IMU, [docs/04](docs/04-lidar-mid360.md)); the position is unverified. The SLAM service did not answer one start request.
* Force units of `FORCE_ACT` (grams?) are assumed, not verified.

## License

MIT, see [LICENSE](LICENSE).
