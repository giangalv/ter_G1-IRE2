# 01 — Network and computers

Measured on 2026-11-04 with a laptop wired to the robot's Ethernet.

## Devices on the LAN (`192.168.123.0/24`)

| IP | What | Reachable services seen |
|---|---|---|
| `192.168.123.99` | the dev laptop (interface `enp59s0`, /24) | — |
| `192.168.123.161` | PC1, Unitree motion-control computer (assumed from the usual G1 layout) | answers ping; **SSH closed** |
| `192.168.123.164` | PC2, **Jetson Orin NX** (hostname `ubuntu`) | ping, **SSH (22)** |
| `192.168.123.120` | **Livox MID-360** LiDAR, head (MAC `ec:72:f7:66:84:f7`) | see [04](04-lidar-mid360.md) |
| `192.168.123.210` | **left** Inspire hand | TCP **6000** open, 502 closed |
| `192.168.123.211` | **right** Inspire hand | TCP **6000** open, 502 closed |

The LAN has no other host; in particular the palm cameras are **not** IP
cameras (they are USB devices of the Jetson, see [03](03-palm-cameras.md)).

Laptop side: `enp59s0` was already configured as `192.168.123.99/24`; no
gateway is needed for the robot LAN.

## The Jetson (PC2, `192.168.123.164`)

* Login: user `unitree` (Unitree's default credentials — ask the lab; not stored here).
* JetPack 5.1.1, Ubuntu 20.04, Python 3.8, OpenCV (cv2) 4.2.0, `gst-launch-1.0`.
  No `ffmpeg`, no `v4l2-ctl`.
* ROS: only **foxy** and **noetic** are installed — no Humble. A Humble stack
  has to run in Docker, and the `unitree` user cannot use docker (permission denied).
* Running services: Unitree `master_service`, `videohub_pc4`. No Inspire service.
* `unitree` is in the `video` group (camera nodes are readable).
* USB devices seen: RealSense D435i (`/dev/video0-5`), the two palm cameras
  (`/dev/video6-9`), WCH serial chips (`ttyACM0-2`, purpose unknown), a WCH
  "USB 10/100 LAN" (`eth1`, no carrier), card reader, Realtek WiFi 6 — **no LiDAR on USB**.
* Livox SDK2 libraries and headers are installed (`/usr/local/lib/liblivox_lidar_sdk_*`)
  but no lidar process runs.

### Network interfaces and Unitree services on the Jetson

* The robot LAN (`192.168.123.164/24`, NetworkManager profile `unitree1`) is on
  the **onboard PCIe NIC** (driver `r8169`). The interface names are **not
  stable across boots**: on 2026-11-04 the onboard NIC was `eth1` and the
  "USB 10/100 LAN" adapter (QinHeng `1a86:5395`, `cdc_ether`, no carrier) was
  `eth0`; earlier in the day it was the other way round. Select by MAC or
  driver, never by name.
* `master_service` (`/unitree/module/master_service`) starts and monitors the
  Unitree services; its log is `/unitree/var/log/master_service/master_service.LOG`.
  A service that fails to stay up is restarted every ~5 s and, after a
  "protect count" of **30**, is **given up for good** until it is started again
  (`mscli startservice <name>`) or the Jetson reboots with the cause fixed.
* `video_hub_pc4` runs `videohub_pc4 /dev/video4`: it reads the **RealSense D435i
  colour stream** (`/dev/video4`, 1920x1080 YUY2 @ 15 fps via `v4l2src`), encodes
  it with the Jetson hardware encoder (`nvv4l2h264enc`, `nvjpegenc`) and
  serves it over DDS — presumably what the phone app shows as video (not
  verified). It uses its **own DDS config**,
  `/unitree/module/video_hub_pc4/cyclonedds.xml`, which pins CycloneDDS to the
  interface **`eth0`** by name.
* **Why the video service is dead — three separate causes, found on 2026-11-04:**
  1. **The DDS config pins `eth0`** (verified). The interface names flipped, so
     `eth0` became the USB LAN adapter (no carrier, no address) and the robot LAN
     moved to `eth1`. Run by hand with the stock config, the hub prints
     `eth0: does not match an available interface` /
     `dds_create_participant: Error` and aborts (exit 134), so `master_service`
     saw it die after every start until its protect count (30) ran out.
     Selecting the interface by address instead,
     `<NetworkInterface address="192.168.123.164" .../>`, fixes this part. The
     kernel names the PCIe NIC (`r8169`) and the USB adapter (`cdc_ether`) in
     probe order, so the names can flip again at any boot.
  2. **The binary hard-codes `eth0` for the app stream** (verified). Its pipeline
     ends in `rtph264pay ! udpsink host=230.1.1.1 port=1720 multicast-iface=eth0`.
     A copy of the binary in `/tmp` with `eth0` replaced by `eth1` (same length, the
     original untouched) sends the 720p H.264 stream as RTP multicast: the laptop
     received it (8 Mbit/s, ~15 frames/s, payload type 96, from `.164`) and the DDS
     topics `/frontvideostream` (`unitree_go/msg/Go2FrontVideoData`),
     `/api/videohub/{request,response}` and `/videohub/inner` appeared.
  3. **The camera's USB 3 link keeps being reset by the kernel — traced to one hub
     port (checked with root on 2026-11-05; fixed by moving the cable, see the result below).** Even with 1 and 2 fixed,
     the hub stops after 26-90 s: either it aborts (`v4l2src0` reports
     `poll error 1: Invalid argument`, then `Failed to allocate a buffer`,
     `streaming stopped, reason error (-5)`, and the program calls `g_error`), or it
     hangs with the process alive (`v4l2src` waits in `poll()`, the per-second sample
     counters go from 15 to 0). It reproduces **without any Unitree software**:
     `gst-launch-1.0` with Unitree's pipeline and `fakesink`s died after 35 s, and
     even `v4l2src ! queue ! nvvidconv ! fakesink` died after 15-78 s, while the camera
     alone (`v4l2src ! fakesink`, 1080p YUY2 15 fps) ran 240 s clean once. What the
     kernel log (`dmesg -T`, root) shows:
     * every failure coincides with `usb 2-2.1: Disable of device-initiated U1 failed`
       / `... U2 failed` and `usb 2-2.1: reset SuperSpeed Gen 1 USB device number N
       using tegra-xusb` (the D435i is `2-2.1`, on the Realtek RTS5411 4-port USB 3.0
       hub `0bda:0411` = `2-2`; the U1/U2 lines suggest it no longer answered control
       requests when the kernel went to reset it, but the cause was not traced);
     * the resets happen **with nothing using the camera**: 43 resets and 3
       disconnects of `2-2.1` in the first 25 minutes after a boot, five of them in a
       single 30 s window of pure idle. No process had a `/dev/video*` or
       `/dev/bus/usb` node open, no new process was started in a 30 s window, the
       `kworker` doing it is `usb_hub_wq`, no Unitree script mentions a USB reset.
       The resets already happened right after boot (17:46) while `master_service`
       was restarting `video_hub_pc4`, before any experiment of mine;
     * the two palm cameras (`1c45:6200`, `1-2.2.1` and `1-2.2.2`) also drop off
       the bus, **both at once** (7 disconnect events in the same 25 minutes);
     * the xHCI root port towards the hub stays `U0`, `over_current_count` is 0 on
       every port, the temperatures are 46-49 °C and the camera enumerates at
       5000 Mbit/s, so the fault is on the hub ↔ camera side.
     Tried and **ruled out**: USB 3 link power management and autosuspend. With
     `usb3_lpm_permit=0` and `power/control=on` on `2-2.1` and on the hub `2-2`
     (non-persistent, restored afterwards) the same pipeline died after 78 s with the
     same reset, and a second run 3 s later after 6 s with a full USB disconnect. Left as the most likely causes, none testable by software: the USB 3
     cable or its connectors (it runs through the neck), the hub, or the D435i unit.
     Cheap physical tests: reseat the camera's USB-C cable at both ends, try another
     USB 3 port or cable, then run the 150 s `gst-launch-1.0 v4l2src ! ... ! nvvidconv
     ! fakesink` test again and count `dmesg -T | grep 'usb 2-2.1: reset'`.
     **Result of the port swap (2026-11-05, 18:30):** the D435i's cable is integrated in
     the head (cannot be replaced) and is labelled USB 3.0. Moved to the connector
     that had been powering the tactile switch (the switch now sits on the USB 2.0
     connector, the hands stayed reachable), the camera enumerates on the **same
     Realtek hub but on its port 3 (`2-2.3`) at 5000 Mbit/s** and is stable: two
     150 s `gst-launch-1.0` runs with Unitree's colour pipeline survived with no
     kernel USB event, then the patched video hub ran 150 s at 14.8-15.0 frames/s,
     8 Mbit/s, without a gap. Only two resets happened, in the first 17 s after
     plugging in. Same camera, same cable, same hub as on port 1 (`2-2.1`), where it
     was reset ~46 times in 30 min: **the fault is the old connector / hub port 1**,
     not the cable or the camera. Side findings: on a direct Jetson port
     (`1-3`, the USB 2.0 root port 3) the camera got no SuperSpeed at all (it enumerated
     at 480 Mbit/s, stable, but 1080p15 YUY2 does not fit; 1280x720@15 ran 100 s); the
     palm cameras sit behind a USB 2.0-only 10-port hub (`1a86:809f`) behind hub
     port 2, which is why they drop together. Longer runs and the phone app have
     not been checked yet.
* **State left on 2026-11-05:** no video service is running; `master_service` has
  given up on `video_hub_pc4` (protect count 30; it still tries every 5 s and fails
  at once) and on `video_hub_pc4_chest`, which has no camera on this unit and died on
  2026-11-04 about a minute after the experiment with the copy started (the link is
  unknown). After the reboot on 2026-11-05 the names are as before: `eth1` = the
  `r8169` NIC with the robot LAN, `eth0` = the `cdc_ether` USB adapter (down), so
  cause 1 still applies. Nothing under `/unitree` was changed and no persistent
  change was made. A reboot, or `mscli startservice video_hub_pc4`, restarts the
  hub with the stock config, which fails as in cause 1.
* **Fix for causes 1 and 2, tried 2026-11-05 and removed (see the test below):**
  `/etc/udev/rules.d/71-g1-robot-lan.rules` (a new file, nothing else was edited):
  ```
  SUBSYSTEM=="net", DRIVERS=="cdc_ether", NAME="ethusb0"
  SUBSYSTEM=="net", KERNELS=="0008:01:00.0", NAME="eth0"
  ```
  The PCIe NIC (`r8169`, PCI `0008:01:00.0`, the robot LAN) is always `eth0`, as the
  image's own `/etc/network/devnames` says (`eth0:Realtek ... PCI Express`), and any
  USB LAN adapter gets `ethusb0`. A `udevadm test` dry run assigned exactly these
  names; the running interfaces were not renamed (they are up), so it takes effect
  at the next boot. The NetworkManager profile `unitree1` is bound to neither a name
  nor a MAC, so it was expected to follow the NIC.
  **Reboot test, 2026-11-05 ~19:00 Jetson time: the Jetson was unreachable over the
  cable after the first two boots with the rule** (more than 11 minutes on `.164`, ARP
  incomplete, no IPv6 link-local, its MACs never seen, while PC1, the LiDAR and both
  hands answered). Recovery needed a monitor and keyboard: the rule was deleted and the
  Jetson rebooted, and then it answered again on the cable. **Cause not established.**
  A photo of the console taken on the *third* boot, with the rule still in place, shows
  `ethusb0` DOWN, `eth0` UP with `192.168.123.164/24` and the NetworkManager profile
  `unitree1` active on `eth0`: on the Jetson side the rename and the profile were
  fine, which contradicts the guess that another profile had taken the NIC. Whether
  the laptop could reach it on that boot was not measured, and the logs of the failed
  boots are gone (the journal is volatile: there is no `/var/log/journal`). So the rule
  is the only change that coincides with the outage, but it is not proven to be the
  cause. The earlier claim that a failed rename would leave things as before was
  never tested.
  Revert: delete the file and reboot. To test again, first make the journal
  persistent (`mkdir -p /var/log/journal`, restart `systemd-journald`), keep a monitor
  and keyboard attached, and watch `192.168.123.164` from the laptop.
  After the rule was removed (boot of 20:15 Jetson time) the names are as before
  (`eth1` = `r8169` with `.164`, `eth0` = the USB adapter, down), the D435i is on
  `2-2.3` at 5000 Mbit/s with no reset, and `video_hub_pc4` runs with the stock config
  but sends **no RTP** to the LAN (0 packets in 15 s: `multicast-iface=eth0` is the
  unplugged USB adapter). The stock `cyclonedds.xml` and the binary
  are unchanged and need no edit once `eth0` is the robot LAN.
  Cause 3 was a faulty connector, fixed by moving the cable (see above).
* `video_hub_pc4_chest` runs on `/dev/video10`, which does not exist on this unit.

## Talking DDS from the laptop

The Unitree stack uses ROS 2 on **domain 0** with **CycloneDDS**. From a
laptop that is wired to the robot LAN:

```bash
unset ROS_LOCALHOST_ONLY            # a shell profile with ROS_LOCALHOST_ONLY=1 hides the robot
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces>
  <NetworkInterface name="enp59s0" multicast="true"/></Interfaces></General></Domain></CycloneDDS>'
ros2 topic list --no-daemon          # the daemon caches the first environment it saw
```

Pitfalls met on the way:

* `ROS_LOCALHOST_ONLY=1` and a non-zero `ROS_DOMAIN_ID` (e.g. 99) in `~/.bashrc`
  make the robot invisible. Cyclone then picks `lo` and shows only local topics.
* Without the Unitree message packages (`unitree_hg`, `unitree_go`, `unitree_api`)
  built on the laptop, `ros2 topic echo` of Unitree topics fails. The message
  definitions are in the official `unitree_ros2` GitHub repository.
* The ROS 2 daemon caches the discovery configuration of the first command;
  use `--no-daemon` or `ros2 daemon stop` after changing the environment.
