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
bash <ter_G1U6>/src/g1_viz/scripts/viz_real.sh wifi # RViz + hands + cameras over the Wi-Fi (G1_WIFI_PROFILE=light|lidar|full)
G1_PW=... G1_CYCLONEDDS_URI=file://$HOME/.cache/g1_wifi_cyclonedds.xml tools/g1_up.sh   # LiDAR service on (needed for lidar|full)
G1_PW=<Jetson password> tools/g1_wifi.sh down      # everything back: cable, lab Wi-Fi, Jetson rules removed
```

`up` copies and starts the relay on the Jetson, sets the root runtime rules (route, proxy ARP, 2 FORWARD rules for
192.168.123.98), scans for `G1-NX-AP-*`, gives the laptop 10.42.0.250/24 (fixed, no DHCP) plus 192.168.123.98/32 on the
profile `g1-ap`, disconnects the cable device and joins the AP; if the robot LAN is not reachable afterwards it goes back
by itself. The password of the AP is read once from the Jetson (read-only sudo) when the profile does not exist yet and
lives only in the NetworkManager profile. Nothing persists on the robot: all lost at its next boot. **The laptop has one
Wi-Fi card: while on the robot's AP it has no internet** (tested unattended, always with a watchdog that runs `down`).

### Viz over the Wi-Fi, measured 2026-11-06 (laptop 2-3 m from the robot, nodes only, no RViz window)

| Profile | joint states | hand tactile (cable: 23 Hz) | palm cameras | LiDAR cloud | RealSense | hands' Modbus |
|---|---|---|---|---|---|---|
| full (everything) | 50 Hz | 0.4 / 0.1 Hz | 6 Hz | 10 Hz | colour 3.9, depth 3.5 Hz | **timeouts** (link saturated) |
| `lidar` (no RealSense) | 50 Hz | 4.5 / 4.2 Hz | 5.9 Hz | 9.9 Hz | - | 10 errors in 50 s |
| `light` (default: no LiDAR, no RealSense) | 50 Hz | **16.5 / 16.1 Hz** | 5.6 Hz | - | - | 4 errors in 50 s |

Why: the offered load is LiDAR 35 Mbit/s + RealSense colour (raw 640x360, ~66) + depth (~24) + cloud, more than the ~60-100
Mbit/s the link really carries, and the hands' Modbus (tiny packets, needs low latency) queues behind it. Modbus on its
own is fine over the Wi-Fi: 100/100 transactions per hand, with or without NAT (source .98 or 10.42.0.250), RTT median
3-12 ms, p99 17-66 ms, 125-register answers 12 ms. The hand bridge's default 20 ms timeout is for the cable: over the
Wi-Fi `viz_real.sh wifi` sets `hand_tcp_timeout_s` 0.15. Ideas to carry more: a compressed RealSense colour stream,
a decimated cloud made on the Jetson (needs its own DDS domain), a Wi-Fi 5 GHz AP (the AP is 2.4 GHz channel 6).
Not tested yet: a walking robot, distance, RViz itself over the Wi-Fi.

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
