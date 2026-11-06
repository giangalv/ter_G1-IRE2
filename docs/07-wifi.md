# 07 — Wi-Fi to the robot (no cable)

## What the robot offers (read 2026-11-05)

* The Jetson's `wlan0` is an **access point**: SSID `G1-NX-AP-026047`, WPA2-PSK, 2.4 GHz channel 6 (20 MHz),
  address **10.42.0.1/24**, NetworkManager profile `Hotspot-41` (`mode ap`, `ipv4.method shared`,
  `autoconnect no`, so something starts it after boot; it was up). The PSK is stored in that profile
  (readable with `sudo nmcli -s -g 802-11-wireless-security.psk connection show Hotspot-41` on the Jetson).
* "shared" means the Jetson hands out 10.42.0.x by DHCP, forwards (`ip_forward=1`) and **masquerades** towards the
  robot LAN: a client on the AP reaches 192.168.123.x, seen as 192.168.123.164. Its default route is
  192.168.123.1 on `eth1`.
* **The SSID changes at every robot boot.** `/usr/local/bin/start_ap.sh`, started by `start_wifi_ap.service`, builds
  `G1-NX-AP-` plus 6 random digits, runs `nmcli device wifi hotspot ... band bg channel 6` and creates a new profile
  `Hotspot-N` each time (that is why the laptop has saved profiles with other numbers, e.g. `G1-NX-AP-017046`,
  `-026047`, `-024737`: all the same robot, there is no other G1). The **password is fixed in that script**, the same at
  every boot (its value is not recorded in this repo). Consequence: find the AP by scanning for `G1-NX-AP-*`
  (`tools/g1_wifi.sh` does), never by a remembered name. Anyone in radio range who knows that password gets into the
  robot network (DDS without authentication, the `bashrunner` service): changing it is a Unitree-side edit, not done.
* The Jetson cannot be an AP and a client at once (one radio): to put the robot on the lab Wi-Fi the AP has to go
  down. The lab network gives public addresses (130.251.x.x) and the robot's DDS has no authentication (and a
  `bashrunner` service that runs commands), so that is **not** recommended.

## Test (2026-11-05, laptop with one Wi-Fi card, cable taken down, unicast DDS peers only)

Signal 82 %, link 130 Mbit/s. The laptop leaves the lab Wi-Fi for the test (it has one card): a script joined the AP,
measured and restored the cable and the lab Wi-Fi, with a watchdog that restores them anyway after 10 min.

| Check | Result |
|---|---|
| ping 10.42.0.1 (30 x 0.2 s) | 0 % loss, 2.1 / 6.3 / 104 ms (min/avg/max) |
| ping .164, .161 (PC1), .120 (LiDAR), .210/.211 (hands) through the Jetson | 0 % loss, 2-4 ms |
| TCP connect to the hands' Modbus port 6000 (.210, .211) through the Jetson | OK in 2-4 ms |
| ssh transfer Jetson -> laptop | 50 Mbit/s (limited by ssh, not a Wi-Fi limit) |
| **DDS domain 0** (Cyclone on the Wi-Fi NIC, peers .161 and .164): `/lf/lowstate`, `/utlidar/imu_livox_mid360`, LiDAR cloud | **0 messages, only 2 topics discovered** |
| **DDS domain 42** (the RealSense / palm camera domain on the Jetson, Fast DDS): palm camera JPEG, depth | **8.1 Hz and 13.9 Hz, works** |

So over the AP: IP reaches everything (hands' Modbus included), the Jetson's own DDS domain works, and the robot's
domain 0 does **not**. The first guess (PC1 has no route back to 10.42.0.0/24 because of the NAT) was **wrong**, see
test 2.

## Test 2 (2026-11-05 17:47, option 1 below): same subnet, no NAT, and it still fails

The laptop got a second address 192.168.123.98/32 on the AP profile. On the Jetson, runtime only and removed again
afterwards: `ip route add 192.168.123.98/32 dev wlan0`, `ip neigh add proxy 192.168.123.98 dev eth1`, and two
`iptables -I FORWARD` ACCEPT rules for that one address (needed: NetworkManager's "shared" rules REJECT every other
forwarded packet to or from `wlan0`). Cyclone bound to `address="192.168.123.98"`, `AllowMulticast=false`, unicast
peers .161 and .164.

| Check | Result |
|---|---|
| ping from source .98 (no NAT) to .164, .161, .120, .210, .211 | 0-10 % loss, 2-4 ms (PC1 and LiDAR had 100-450 ms outliers) |
| DDS domain 0: `/lf/lowstate`, LiDAR IMU, cloud, `/lowstate`, topic list | **0 messages, 2 topics** (same as before) |
| DDS domain 42: palm camera JPEG / RealSense depth | 7.4 Hz / 0 msgs (depth: not understood, the driver may have been busy) |

So the network path is fine and the NAT was not the problem. The cause is **discovery**, and it reproduces on the
cable, with the laptop on 192.168.123.99 (`/lf/lowstate`, 7-10 s, domain 0):

| Cyclone options on the laptop | Result |
|---|---|
| default (multicast) | 20 Hz, works |
| interface `address`, `AllowMulticast=false`, peers .161/.164 | **0 msgs** |
| the same with explicit peer ports `host:7410, 7412 ... 7428` | **0 msgs** |
| `AllowMulticast=false`, no interface, peers | **0 msgs** |
| interface `address`, `AllowMulticast=spdp`, peers | 20 Hz, works |
| interface `address`, multicast on, peers | 20 Hz, works |

Reading: the robot's DDS (PC1) only learns about a new participant from its **multicast SPDP**; unicast SPDP sent
to it is not enough (not understood why: its receive socket for the discovery port, or a fixed participant index).
Once the multicast SPDP has been heard, everything else (data, replies) is unicast and works. A multicast packet
from the laptop on the AP is never forwarded by the Jetson to `eth1`, so PC1 never hears it.

## Test 3 (2026-11-06 11:07): the multicast SPDP relay makes DDS domain 0 work over Wi-Fi

`tools/wifi_dds_relay.py` runs on the Jetson as user `unitree` (no root, standard library only). It joins the RTPS
discovery group 239.255.0.1 port 7400+250*domain on `wlan0` and on `eth1` and copies every packet to the other side
(`IP_MULTICAST_LOOP=0`, `IP_MULTICAST_ALL=0`, so nothing loops). The payload of an SPDP packet carries the sender's unicast
locators (192.168.123.98), so PC1 answers straight to .98, and that path is the one test 2 already showed to work (route
`.98/32 dev wlan0` + proxy ARP + two FORWARD ACCEPT rules on the Jetson). The laptop's Cyclone uses
`<NetworkInterface address="192.168.123.98"/>` and `<AllowMulticast>spdp</AllowMulticast>` (multicast for discovery only)
with peers .161 and .164. Self-test on the cable before: test packets out of `wlan0` arrived on the cable, PC1's real
SPDP packets crossed the other way.

| Check (laptop 2-3 m from the robot, 11:08) | Result |
|---|---|
| ping from .98 to .164, .161, .120, .210, .211 | 0 % loss, 2-6 ms |
| `/lf/lowstate` | 239 msgs, **19.9 Hz** |
| `/utlidar/imu_livox_mid360` | 2368 msgs, **197 Hz** |
| topics seen in domain 0 | **124** (was 2) |
| LiDAR cloud, **full rate** (~35 Mbit/s offered) | 80 msgs in 8 s, **10.0 Hz**, 20180 points |
| `/lowstate` (1 kHz) | 5464 msgs in 6 s, **910 Hz** |
| domain 42 palm camera JPEG | 8.1 Hz |
| domain 42 RealSense depth | 0 msgs: **the driver was broken, not the Wi-Fi** (0 on the cable too, see docs/06) |

The relay saw about 9 packets/s from the robot LAN and ~0.4/s from the laptop. Not yet tried: a walking robot (signal
changes with the distance), the viz stack over Wi-Fi, long runs.

Order of attempts that led here, for the record: test 1 (NAT, 0 msgs), test 2 (same subnet, 0 msgs), cable experiments
(only multicast SPDP gets PC1 to talk), test 3 (relay: works). Two earlier runs of test 3 were void: the AP had a new
name after a robot reboot (see above), and my relay start command killed itself (`pkill -f PATTERN` also matches the
shell whose command line contains PATTERN: use a pid file).

## Using it: `tools/g1_wifi.sh`

```
G1_PW=<Jetson password> tools/g1_wifi.sh up        # robot on, cable still connected for the first time after a robot boot
G1_WIFI_PROFILE=nav bash ros2_ws/src/g1_viz/scripts/viz_real.sh wifi   # RViz over the Wi-Fi; profile nav | manip (default) | full
G1_PW=... G1_CYCLONEDDS_URI=file://$HOME/.cache/g1_wifi_cyclonedds.xml tools/g1_up.sh   # LiDAR service on (needed for nav|full)
G1_PW=<Jetson password> tools/g1_wifi.sh down      # everything back: cable, lab Wi-Fi, Jetson rules removed
```

`up` copies and starts the relay on the Jetson, sets the root runtime rules (route, proxy ARP, 2 FORWARD rules for
192.168.123.98), scans for `G1-NX-AP-*`, gives the laptop 10.42.0.250/24 (fixed, no DHCP) plus 192.168.123.98/32 on the
profile `g1-ap`, disconnects the cable device and joins the AP; if the robot LAN is not reachable afterwards it goes back
by itself. The password of the AP is read once from the Jetson (read-only sudo) when the profile does not exist yet and
lives only in the NetworkManager profile. Nothing persists on the robot: all lost at its next boot. **The laptop has one
Wi-Fi card: while on the robot's AP it has no internet** (tested unattended, always with a watchdog that runs `down`).

### Viz over the Wi-Fi: what to show depends on the task (profiles, measured 2026-11-06)

The link carries roughly 60-100 Mbit/s and the offered load is more than that (LiDAR 35 Mbit/s, RealSense raw colour ~66,
depth ~24, cloud ~8 after decimation), so a profile picks what is on, as the task changes:

* **`nav`** (walking around): LiDAR + head camera (colour, depth cloud), no hands and no palm cameras.
* **`manip`** (default; manipulating): hands (tactile) + palm cameras + head camera, no LiDAR.
* **`full`**: everything; does not fit (see the table).

`G1_WIFI_PROFILE=nav|manip|full tools... viz_real.sh wifi` starts the matching nodes and RViz config
(`g1_real_nav.rviz`, `g1_real_manip.rviz`, `g1_real.rviz`). Two things make this cheap to switch:

* **Head colour as JPEG and depth as PNG from the Jetson** (`tools/realsense_encoder_node.py`, node `realsense_encoder`; colour 10 Hz, quality 80,
  ~21 kB per frame = 1.2-1.7 Mbit/s instead of 66). The relay on the laptop decodes it. It encodes only while someone
  subscribes. Started with the driver (`realsense_ros2.sh start`, also by the supervisor and by `autostart`).
* **Lazy relays** (`lazy:=true` in `realsense_relay` and `hand_camera_relay`): a stream is pulled from the Jetson only
  while somebody in the laptop's domain subscribes to its output, and released 5 s after the last subscriber leaves. So
  ticking / unticking a display in RViz starts / stops that traffic; `nobody listening` costs nothing on the Wi-Fi.
  (Raw LiDAR and tactile are not relayed: RViz or the hand bridge subscribe to them directly.)

Measured with the nodes only (no RViz window), laptop 2-3 m from the robot, 15 s per topic:

| Profile | joint states | hand tactile (cable: 23 Hz) | palm cameras | LiDAR cloud | head colour / depth cloud | hand bridge |
|---|---|---|---|---|---|---|
| `nav` | 50 Hz | - | - | 9.8 Hz | 3.5 / 4.0 Hz | - |
| `manip` | 50 Hz | 6.8 / 8.5 Hz | 5.7 / 6.0 Hz | - | **7.3** / 4.1 Hz | 7 errors in ~60 s |
| `full` | 50 Hz | 2.0 / 3.4 Hz | 5.8 / 5.6 Hz | 9.9 Hz | 3.2 / 1.9 Hz | 6 errors in ~60 s |

Earlier runs, before the JPEG colour (colour was the raw 66 Mbit/s): everything on gave tactile 0.4 / 0.1 Hz and
Modbus timeouts; LiDAR + hands without RealSense gave tactile 4.5 Hz; hands + cameras with neither LiDAR nor RealSense
gave tactile 16.5 Hz.

**Same test twice more, 2026-11-06 afternoon, after a real robot boot, two laptop cards** (run 1 / run 2; the table above is the
morning run, single card):

| Phase | joint states | LiDAR | head colour / depth cloud | tactile | hand bridge errors |
|---|---|---|---|---|---|
| `nav` run 1 | 50 Hz | 7.3 Hz (lowstate 15.7 Hz) | 3.8 / 2.7 Hz | - | - |
| `nav` run 2 | 50 Hz | 9.9 Hz | 5.5 / 3.9 Hz | - | - |
| `manip` run 1 | 50 Hz | - | 3.2 / 4.1 Hz | 3.9 / 3.6 Hz | 17 in ~60 s |
| `manip` run 2 | 50 Hz | - | **0 / 0** (RealSense stalled) | **16.3 / 11.4 Hz** | 12 in ~60 s |
| `full` run 1 / 2 | 50 Hz | 9.9 Hz | 4.0 / 3.5 and 3.7 / 3.5 Hz | 0.3-0.7 Hz | 16 |

Read them like this. (1) The numbers move from run to run by a factor of 2 (the 2.4 GHz channel is shared with the
building: the AP's ping shows ~3 % loss and max 100-200 ms in every run), so do not read more into one run than the
order of magnitude. (2) In `manip` run 2 the RealSense had stalled (its supervisor restarted it at 21:04:56 Jetson time,
~1.5 min later; cause not found) and with no head camera the tactile came back to 16 / 11 Hz: **the head camera is what
costs the hands their rate.** What crosses the link for it is the raw depth image from the Jetson (424x240 Z16 = 203 kB per
frame, ~24 Mbit/s at 15 Hz); the cloud is built on the laptop afterwards and costs no link (an earlier version of this
text said the cloud's size was the cost: wrong). That step is done now, see "Compression" below. (3) In `nav` the colour is slower than in `manip` because the LiDAR shares the
link.
`lazy` verified in all runs: with nobody listening 0 pulls started; the probe started both RealSense groups
and they were released 5 s after it ended. On the cable: colour 4.7 Hz, depth 7.4 Hz through the same path.

### Compression of the head camera: what it is and how to choose

Two streams of the D435i are compressed **on the Jetson** by `tools/realsense_encoder_node.py` (started with the driver) and
only while the laptop's relay subscribes (the relay itself subscribes only while an RViz display / node uses the output):

| Stream | Raw from the driver | Compressed (default) | How |
|---|---|---|---|
| colour 640x360 | 66 Mbit/s (15 Hz) | **JPEG q80, 10 Hz: ~21 kB/frame = 1.7 Mbit/s**, lossy | `/camera/color/image_raw/compressed` |
| depth 424x240 Z16 | 204 kB/frame, 22.7 Mbit/s (14 Hz) | **PNG 16 bit level 1, 5 Hz: 46 kB/frame = 1.8 Mbit/s**, lossless | `/camera/depth/image_rect_raw/compressedDepth` |
| depth, level 6, 8 Hz | | 37.7 kB/frame = 2.4 Mbit/s | |

The PNG is decoded on the laptop to exactly the raw frame (78 of 78 frames identical when compared stamp by stamp); the
relay then makes the same `/g1/camera/depth/image` (32FC1, m) and `/g1/camera/depth/points` as with raw, so RViz configs
do not change. The format is the image_transport `compressedDepth` layout, but it was only decoded by our own code:
Humble's own `image_transport republish` fails with that plugin whatever the content ("subscribeImpl with five arguments
has not been overridden").

**Where each choice is made**

* **On the laptop, what to take** (`viz_real.sh`, or the launch args `rs_color_compressed` / `rs_depth_compressed`):
  `G1_RS_COLOR=jpeg|raw` and `G1_RS_DEPTH=png|raw`. Default: jpeg + png on `wifi`, raw on `cable`. Use `raw` on the cable
  (nothing to save, full 15 Hz and no loss in the colour).
* **On the Jetson, how much** (live, no restart, any time): `tools/realsense_ros2.sh stream` shows the values and
  `tools/realsense_ros2.sh stream depth_rate_hz=10 depth_png_level=6 color_rate_hz=5 color_quality=60` changes them
  (`ros2 param set /realsense_encoder ...` there does the same). They go back to the defaults (colour 10 Hz q80, depth
  5 Hz level 1) whenever the driver is restarted by `start`, by the supervisor or by a reboot; to change the defaults edit
  `realsense_encoder_node.py` and run `tools/realsense_ros2.sh start`.
* **Rules of thumb**: the traffic is `kB per frame x rate`. Depth rate is the strongest lever for the hands' tactile rate (the
  head camera costs the hands, see above): 5 Hz = 1.8 Mbit/s, 10 Hz = 3.7, 15 Hz = 5.5 (all lossless; the raw stream was 22.7).
  A higher PNG level saves ~18 % more at the price of Jetson CPU (not measured). JPEG quality 60 instead of 80 roughly
  halves the colour (not measured). The cloud rate (`points_rate_hz`, 5) and the image rate (`image_rate_hz`, 10) of the
  relay only limit what is published on the laptop; they cannot go above what the Jetson sends.
* The cloud stays at about 2.7-3 Hz on this laptop (it is built from every 2nd pixel and published as a 240 kB message,
  also on the cable run); not a Wi-Fi limit.

**Measured with PNG depth + JPEG colour** (2026-11-06 14:22, two cards, AP ping 0 % loss, 2.6 ms; run 3):
`nav`: colour 6.9 Hz, depth cloud 2.7 Hz, LiDAR 9.7 Hz, lowstate 19.7 Hz, no hand errors. `manip`: colour 4.7 Hz, cloud 3.1 Hz,
**tactile 6.1 / 14.8 Hz** (3.9 / 3.6 with raw depth in run 1), palm cameras 5.7 / 5.9 Hz, **4 hand bridge errors** in ~60 s
(12-17 before). `full`: the RealSense was dead for that phase (colour and depth 0 messages; see 06, its supervisor restarted
it at 21:25 Jetson time), LiDAR 9.9 Hz, tactile 2.9 / 2.7 Hz, 10 errors.

### Two Wi-Fi cards in the laptop (2026-11-06): the lab keeps its internet

With a USB dongle plugged in, the lab Wi-Fi profile moves to the dongle (`nmcli connection modify <lab> connection.interface-name
<dongle>`) and `g1_wifi.sh up` takes the Wi-Fi device that has no connection, so the internet stays while the laptop is on the
robot. Tested with a D-Link N150 (rt2800usb, 2.4 GHz only) for the lab and the laptop's Intel AX201 for the robot. Does the dongle
disturb the robot link when both are on 2.4 GHz (lab on channel 11, robot AP on 6, a few cm apart)? Back-to-back, twice each:
**no**: 30 MB over ssh through the AP 50-51 Mbit/s with the dongle connected and 49-52 without, 0 % ping loss in both, link
rate 130 Mbit/s, signal 73-75. (Whatever the ssh transfer measures, 50 Mbit/s is not the radio's limit and not what
the viz streams reach: ssh encryption on the Jetson is a likely cap.) A dongle with 5 GHz would also leave the lab on 5 GHz.

### AP on 5 GHz: tried 2026-11-06, hung the Jetson (do not repeat in place)

With the user's OK, on the cable: `nmcli connection modify Hotspot-42 802-11-wireless.band a 802-11-wireless.channel 36` and
`nmcli connection up` ("successfully activated"). The next `iw dev wlan0 info` in the same ssh session never returned and from
then on the Jetson's sshd gave no banner ("timed out during banner exchange", later "Exceeded MaxStartups" from the piled-up
attempts) while ping, DDS (RealSense, palm cameras) and PC1 kept working and the AP was gone from the air. The user power
cycled the robot; the AP came back by itself on 2.4 GHz channel 6 with a new name. Suspected, not verified: the Realtek USB
driver (rtl8852bu) hung on the band switch, taking netlink users down with it. 5 GHz channels allowed to start an AP on
this adapter: 36 and 149 only (the others are "no IR"). If it is tried again: with someone next to the robot, and by creating
the hotspot from scratch with `band a channel 36` (the way `start_ap.sh` creates it at boot), not by changing the live one.

Why the profiles are needed even with a second laptop Wi-Fi card (internet + robot at the same time): it keeps the
internet, the bandwidth of the robot link stays the same. A USB Wi-Fi dongle on the **robot** would not help either:
its Realtek USB card is already the AP; the limit is the 2.4 GHz channel 6, 20 MHz that Unitree's `start_ap.sh` sets at
every boot (changing it is a persistent root change: not done; the attempt above was runtime only).
The hands' Modbus on its own is fine over the Wi-Fi: 100/100 transactions per hand, with or without NAT, RTT median
3-12 ms, p99 17-66 ms, 125-register answers 12 ms. The hand bridge's default 20 ms timeout is for the cable: over the
Wi-Fi `viz_real.sh wifi` sets `hand_tcp_timeout_s` 0.15 (still some errors under load, none with the hands off).
Not tested yet: a walking robot, distance, RViz itself over the Wi-Fi (displays and their enabled state are in the
configs; `g1_real_manip.rviz` has the LiDAR off, `g1_real_nav.rviz` the tactile off).

`g1_wifi.sh up/down` checks the internet with a TCP connect to 1.1.1.1:443, not ping: the lab network drops ICMP to
the outside, which made `down` say "no internet yet" although it was back.

## Ways out (1 and the relay of test 3 done, 2 and 3 not needed for now)

1. **Make the laptop look like it is on the robot LAN** (tried, route/ARP/ping fine, DDS discovery still fails; runtime only, root on the Jetson): give the laptop's Wi-Fi a
   second address 192.168.123.98, on the Jetson `ip route add 192.168.123.98/32 dev wlan0` and
   `ip neigh add proxy 192.168.123.98 dev eth1`, Cyclone on `address="192.168.123.98"` with unicast peers. PC1 then
   answers a neighbour on its own subnet and no NAT is involved. Lost at the next boot.
2. **Relay on the Jetson**: it has `~/cyclonedds_ws` (CycloneDDS 0.10.2 + rmw_cyclonedds_cpp for Foxy, the build
   Unitree asks for), so a Foxy process there can join domain 0 and republish selected, decimated topics into domain
   42 (message types needed on the Jetson: standard ones for the LiDAR, `unitree_hg` for `/lf/lowstate`).
3. Use only what already works: hands (Modbus TCP) and the Jetson's domain 42 (cameras, RealSense).

Data volume: the LiDAR cloud at full rate is ~35 Mbit/s (20k points x 22 B x 10 Hz), too much for this AP: leave it
off or decimate it. Hands' tactile ~1 Mbit/s, `/lf/lowstate` well under 1 Mbit/s, palm JPEGs ~2.5 Mbit/s.
