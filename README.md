# ter_G1-IRE2 — the Unitree G1 U6 with Inspire RH56E2 hands, explained

Basic reference for the lab's **Unitree G1 EDU Ultimate (U6)** humanoid: which
computers it has, how they are networked, what the **Inspire RH56E2** hands
speak, where the **palm cameras** and the **LiDAR** show up, and how to talk to
all of it from a laptop. Everything here was measured on the real robot
(hanging on its support stand) on **2026-11-04**; each page says what is
verified and what is only a hypothesis.

> **Safety first.** The robot was on its stand and *nothing was ever written to
> the hands or the arms* while collecting this. All tools in `tools/` are
> read-only except where stated. Fingers are locked when the hand is powered:
> never pull them by force.

## Contents

| Page | What it covers |
|---|---|
| [docs/01-network-and-computers.md](docs/01-network-and-computers.md) | Devices on the `192.168.123.0/24` LAN, the Jetson, how to connect a laptop, DDS environment |
| [docs/02-inspire-rh56e2-hands.md](docs/02-inspire-rh56e2-hands.md) | Hand model, Modbus TCP on port 6000, **byte-addressed** registers, register map, left/right, tactile status |
| [docs/03-palm-cameras.md](docs/03-palm-cameras.md) | The two palm-facing USB cameras: devices, stable names, left/right mapping |
| [docs/04-lidar-mid360.md](docs/04-lidar-mid360.md) | Livox MID-360 in the head: address, what was tried, why no point cloud yet |
| [docs/05-unitree-dds.md](docs/05-unitree-dds.md) | Native Unitree topics seen over DDS (arms, hands, SLAM API) |
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
| Hand protocol | **Modbus TCP**, port **6000**, unit id **1** (not serial/RS-485 here) |
| Hand IPs | **right = .211, left = .210** (found by experiment, not the other way round) |
| Palm cameras | 2 × UVC "Abham Image", 640×480 @ 30 fps, on the **Jetson's USB hub** (not on Ethernet) |
| LiDAR | Livox MID-360, Ethernet, `192.168.123.120` — currently **not streaming** |
| Head camera | Intel RealSense D435i on the Jetson |
| Robot middleware | Unitree stack on ROS 2 DDS **domain 0** (CycloneDDS) |

## Open questions

* Tactile (`T1`) register block of the hands is located but **not decoded**.
* The **write path** of the hands (set angle / force) is **untested** on hardware.
* The LiDAR point cloud is not flowing; the SLAM service did not answer one start request.
* Force units of `FORCE_ACT` (grams?) are assumed, not verified.

## License

MIT, see [LICENSE](LICENSE).
