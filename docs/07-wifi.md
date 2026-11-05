# 07 — Wi-Fi to the robot (no cable)

## What the robot offers (read 2026-11-05)

* The Jetson's `wlan0` is an **access point**: SSID `G1-NX-AP-026047`, WPA2-PSK, 2.4 GHz channel 6 (20 MHz),
  address **10.42.0.1/24**, NetworkManager profile `Hotspot-41` (`mode ap`, `ipv4.method shared`,
  `autoconnect no`, so something starts it after boot; it was up). The PSK is stored in that profile
  (readable with `sudo nmcli -s -g 802-11-wireless-security.psk connection show Hotspot-41` on the Jetson).
* "shared" means the Jetson hands out 10.42.0.x by DHCP, forwards (`ip_forward=1`) and **masquerades** towards the
  robot LAN: a client on the AP reaches 192.168.123.x, seen as 192.168.123.164. Its default route is
  192.168.123.1 on `eth1`.
* Another saved Wi-Fi profile on the lab laptop, `G1-NX-AP-017046`, is **a different robot's** AP.
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
domain 0 does **not** through the NAT. Likely cause (not verified): PC1 learns the laptop's address 10.42.0.239 from
the discovery data and has no route back to 10.42.0.0/24 (its gateway is not the Jetson). The Jetson is directly on
that network, which is why domain 42 works.

## Ways out (none tried yet)

1. **Make the laptop look like it is on the robot LAN** (runtime only, root on the Jetson): give the laptop's Wi-Fi a
   second address 192.168.123.98, on the Jetson `ip route add 192.168.123.98/32 dev wlan0` and
   `ip neigh add proxy 192.168.123.98 dev eth1`, Cyclone on `address="192.168.123.98"` with unicast peers. PC1 then
   answers a neighbour on its own subnet and no NAT is involved. Lost at the next boot.
2. **Relay on the Jetson**: it has `~/cyclonedds_ws` (CycloneDDS 0.10.2 + rmw_cyclonedds_cpp for Foxy, the build
   Unitree asks for), so a Foxy process there can join domain 0 and republish selected, decimated topics into domain
   42 (message types needed on the Jetson: standard ones for the LiDAR, `unitree_hg` for `/lf/lowstate`).
3. Use only what already works: hands (Modbus TCP) and the Jetson's domain 42 (cameras, RealSense).

Data volume: the LiDAR cloud at full rate is ~35 Mbit/s (20k points x 22 B x 10 Hz), too much for this AP: leave it
off or decimate it. Hands' tactile ~1 Mbit/s, `/lf/lowstate` well under 1 Mbit/s, palm JPEGs ~2.5 Mbit/s.
