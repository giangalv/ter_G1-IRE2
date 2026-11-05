# 07 — Wi-Fi to the robot (no cable)

## What the robot offers (read 2026-11-05)

* The Jetson's `wlan0` is an **access point**: SSID `G1-NX-AP-026047`, WPA2-PSK, 2.4 GHz channel 6 (20 MHz),
  address **10.42.0.1/24**, NetworkManager profile `Hotspot-41` (`mode ap`, `ipv4.method shared`,
  `autoconnect no`, so something starts it after boot; it was up). The PSK is stored in that profile
  (readable with `sudo nmcli -s -g 802-11-wireless-security.psk connection show Hotspot-41` on the Jetson).
* "shared" means the Jetson hands out 10.42.0.x by DHCP, forwards (`ip_forward=1`) and **masquerades** towards the
  robot LAN: a client on the AP reaches 192.168.123.x, seen as 192.168.123.164. Its default route is
  192.168.123.1 on `eth1`.
* The same robot has also been seen with the AP name `G1-NX-AP-017046` (a saved profile on the lab laptop): it is the
  **same robot, the AP was renamed** (017046 -> 026047). There is no other G1 in the lab. Read the current name from
  the Jetson (`nmcli -g 802-11-wireless.ssid connection show Hotspot-41`) instead of trusting an old profile.
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

## Next idea (not tried): carry the multicast SPDP across the Jetson

Run on the Jetson a small user-space relay (no root needed to join a group and send on `eth1`): receive
`239.255.0.1:7400+250*domain` on `wlan0` and re-send it on `eth1`, and the other way round. The payload of the SPDP
packet carries the participant's unicast locators (192.168.123.98), so PC1 would answer straight to .98 over the
routed path that test 2 showed to work. Needs the same root runtime setup as test 2 (route, proxy ARP, FORWARD
rules) and the laptop on `AllowMulticast=spdp`. Alternative: option 2 (Cyclone relay on the Jetson).

## Ways out (1 tried, see test 2)

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
