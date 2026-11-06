# ros2_ws — RViz2 of the real G1 from a laptop (cable or Wi-Fi), ready to use

A ROS 2 **Humble** workspace for the laptop. It shows the **real** robot in RViz2, **read-only** (it only subscribes, nothing is
sent to the robot): the G1 with its joints, both Inspire hands with the tactile cells, the LiDAR cloud, the head camera
(colour + depth cloud), the palm cameras and a flat 2D view of the hands. The same launch works over the **Ethernet cable** and
over the **robot's own Wi-Fi access point**, where a few things are done differently so that the link is not saturated
([docs/07-wifi.md](../docs/07-wifi.md)).

Everything was measured on the real robot. Read [docs/01](../docs/01-network-and-computers.md) once for the network.

## Quick start (about 10 minutes, mostly downloads)

```bash
# 0. Ubuntu 22.04 + ROS 2 Humble (ros-humble-desktop). setup.sh prints the apt line if something else is missing.
cd ros2_ws
bash setup.sh            # downloads Unitree's robot model + message definitions (pinned commits), builds, runs the unit tests
source /opt/ros/humble/setup.bash && source install/setup.bash
```

**On the cable** (laptop on the robot LAN, e.g. `192.168.123.99/24`, see docs/01):

```bash
G1_NIC=<your ethernet NIC> bash src/g1_viz/scripts/viz_real.sh cable         # default NIC name: enp59s0
```

**Over the robot's Wi-Fi** (the laptop leaves the cable; details and limits in docs/07):

```bash
G1_PW=<Jetson password> ../tools/g1_wifi.sh up                               # sets up the Jetson and joins the robot's AP
G1_WIFI_PROFILE=manip bash src/g1_viz/scripts/viz_real.sh wifi               # manip (hands) | nav (LiDAR) | full
G1_PW=<Jetson password> ../tools/g1_wifi.sh down                             # everything back
```

The robot must be on, with its **LiDAR service started after every robot boot**: `G1_PW=<Jetson password> ../tools/g1_up.sh`
(over Wi-Fi add `G1_CYCLONEDDS_URI=file://$HOME/.cache/g1_wifi_cyclonedds.xml`). With one Wi-Fi card the laptop has no internet
while it is on the robot's AP; with a second one (USB dongle) it keeps it ([docs/07](../docs/07-wifi.md)).

### One-time setup of the robot's Jetson (needs ssh to `unitree@192.168.123.164`, `G1_PW`)

```bash
../tools/realsense_ros2.sh install && ../tools/realsense_ros2.sh autostart on       # head camera driver, restarts itself
../tools/hand_cameras_ros2.sh install && ../tools/hand_cameras_ros2.sh autostart on # palm cameras
```

Both live in the Jetson's home (`~/g1_realsense`, `~/g1_hand_cameras`) and start again at every robot boot; nothing outside
the home is touched. Skip what you do not need: `G1_REALSENSE=false` / `G1_HAND_CAMERAS=false` / `G1_HANDS=false` leave the
corresponding part out of the viz.

## What is in `src/`

| Package | What it is |
|---|---|
| `g1_viz` | The viewer: launch file, RViz configs (cable, `manip`, `nav`, hands), joint-state relay, tactile markers, 2D hand view, `scripts/viz_real.sh`. Its [README](src/g1_viz/README.md) lists every option. |
| `g1_unitree_bridge` | Read-only bridges: `/lf/lowstate` to joint states, RealSense and palm-camera relays (domain 42 to domain 0, lazy, JPEG / PNG decoding), depth to cloud. |
| `g1_action` | Contains `inspire_hand_bridge_node` (Modbus TCP to the hands, tactile) and `inspire_protocol` that the viewer uses. The rest of the package (action executor, safety) belongs to the VLA project and is not needed here. |
| `g1_interfaces` | Messages used by `g1_action`. |
| `g1_hand_camera` | The palm-camera node. It runs **on the Jetson** (ROS 2 Foxy); `tools/hand_cameras_ros2.sh` copies it there. |
| `g1_description` | Only a script: `fetch_assets.sh` downloads Unitree's G1 URDFs and meshes (about 60 MB, not stored in git). |

`make setup | build | test | viz_cable | viz_wifi` in this folder are shortcuts for the commands above.

## When something does not show

* **Nothing / no robot in RViz**: the shell you run `ros2 topic list` in must have `ROS_LOCALHOST_ONLY=0`, `ROS_DOMAIN_ID=0`,
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and the Cyclone config of `viz_real.sh` (a `ROS_LOCALHOST_ONLY=1` or another
  `ROS_DOMAIN_ID` hides the robot). `viz_real.sh` sets these for the nodes it starts; `../tools/g1_wifi.sh env` prints them for
  Wi-Fi, docs/01 for the cable.
* **Robot model missing** (grey boxes / error about meshes): `fetch_assets.sh` was not run, or `colcon build` ran before it.
* **No LiDAR**: the robot's `lidar_driver` service is off after every boot, run `../tools/g1_up.sh`.
* **No head or palm camera**: the Jetson drivers are not running (`../tools/realsense_ros2.sh status`, `check`;
  `../tools/hand_cameras_ros2.sh status`) or `G1_REALSENSE` / `G1_HAND_CAMERAS` is `false`.
* **Cyclone refuses to start** with a receive-buffer message: do not put `SocketReceiveBufferSize` in your own config (a laptop
  without root cannot grant it).
* **RViz window freezes** (seen twice when started through `ros2 launch`, cause unknown; RViz started alone did not): close it and
  start again. The viz is read-only, nothing on the robot is affected.
* The robot's clock was about 5 s off the laptop's when measured; it did not matter for the viz (not investigated further).

## License

The scripts and docs of this repository are MIT ([LICENSE](../LICENSE)). The ROS packages in `src/` keep the Apache-2.0
declaration in their `package.xml`. Unitree's files are downloaded by `fetch_assets.sh` under their own license.
