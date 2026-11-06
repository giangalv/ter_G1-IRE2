#!/bin/bash
# Work with the robot over its own Wi-Fi access point instead of the cable (docs/07-wifi.md, test 3).
#
#   G1_PW=<Jetson password> tools/g1_wifi.sh up        # set up the Jetson, leave the cable, join the AP
#   G1_PW=<Jetson password> tools/g1_wifi.sh down      # undo everything: cable and lab Wi-Fi back
#   tools/g1_wifi.sh status
#   tools/g1_wifi.sh env                               # export lines for DDS domain 0 over the Wi-Fi
#
# What `up` does (nothing persists on the robot: it is all lost at the next robot boot, and `down` removes it):
#  * on the Jetson, as root through sudo, runtime only: route `<ADDR>/32 dev wlan0`, proxy ARP for <ADDR> on the LAN
#    side, two narrow iptables FORWARD ACCEPT rules for <ADDR> (NetworkManager's "shared" rules reject the rest);
#  * on the Jetson, as user: tools/wifi_dds_relay.py (copies the DDS discovery multicast between wlan0 and the LAN, because
#    the robot's PC1 only discovers a participant from its multicast packet);
#  * on this laptop: NetworkManager profile `g1-ap` (SSID found by scanning G1-NX-AP-*, because the robot picks a new
#    random name at every boot; the password is the same each time), the extra address <ADDR> (default 192.168.123.98) on
#    the robot LAN's subnet and a fixed 10.42.0.250 on the AP's own subnet (no DHCP); the cable is disconnected so
#    that the Wi-Fi path is the only one.
#  * THIS LAPTOP HAS ONE WI-FI CARD: while connected to the robot it has no lab Wi-Fi and no internet.
#
# The first `up` after a robot boot needs the cable (or a laptop that already knows the password): it reads the
# AP's name and password from the Jetson (read-only sudo) and stores them only in the NetworkManager profile.
#
# Environment: G1_PW (Jetson password, ssh and sudo), G1_NIC (cable NIC, default enp59s0), G1_WLAN (default: the first
# Wi-Fi device), G1_WIFI_ADDR (default 192.168.123.98), G1_WIFI_STATIC (default 10.42.0.250, empty = DHCP),
# G1_JETSON_SSH (override the ssh command, default: password ssh as unitree@192.168.123.164 with askpass.sh).
set -u
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
JETSON_IP=192.168.123.164
AP_IP=10.42.0.1
ADDR=${G1_WIFI_ADDR:-192.168.123.98}
STATIC=${G1_WIFI_STATIC-10.42.0.250}
CABLE=${G1_NIC:-enp59s0}
WLAN=${G1_WLAN:-$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi"{print $1; exit}')}
PROFILE=g1-ap
STATE=${XDG_CACHE_HOME:-$HOME/.cache}/g1_wifi.state
CYCLONE=${XDG_CACHE_HOME:-$HOME/.cache}/g1_wifi_cyclonedds.xml

say() { echo "[g1_wifi] $*"; }
die() { echo "[g1_wifi] ERROR: $*" >&2; exit 1; }

if [ -n "${G1_PW:-}" ]; then
    export SSH_ASKPASS="$HERE/askpass.sh" SSH_ASKPASS_REQUIRE=force DISPLAY=${DISPLAY:-:0}
fi
JETSON_SSH=${G1_JETSON_SSH:-ssh -o StrictHostKeyChecking=accept-new -o PreferredAuthentications=password -o PubkeyAuthentication=no -o ConnectTimeout=8 unitree@$JETSON_IP}

# run a script on the Jetson; the password goes first on stdin (read by the remote shell), the script follows
remote() {
    { printf '%s\n' "${G1_PW:-}"; cat; } | $JETSON_SSH 'IFS= read -r PW; export PW; bash -s -- '"$*"
}

jetson_script() {   # $1 = up | down | ap
    cat <<'EOF'
LAN=$(ip -o -4 addr show | awk '/192\.168\.123\.164\//{print $2; exit}')
p() { printf '%s\n' "$PW" | sudo -S -p '' "$@" 2>&1; }
ACTION=$1; ADDR=$2
case "$ACTION" in
up)
    [ -n "$LAN" ] || { echo "NO-LAN-INTERFACE"; exit 1; }
    p ip route replace "$ADDR/32" dev wlan0
    p ip neigh replace proxy "$ADDR" dev "$LAN"
    p iptables -C FORWARD -s "$ADDR" -i wlan0 -j ACCEPT >/dev/null 2>&1 || p iptables -I FORWARD 1 -s "$ADDR" -i wlan0 -j ACCEPT
    p iptables -C FORWARD -d "$ADDR" -o wlan0 -j ACCEPT >/dev/null 2>&1 || p iptables -I FORWARD 2 -d "$ADDR" -o wlan0 -j ACCEPT
    mkdir -p ~/g1_wifi; cd ~/g1_wifi || exit 1
    [ -f relay.pid ] && kill "$(cat relay.pid)" 2>/dev/null
    (setsid nohup python3 wifi_dds_relay.py "$(ip -o -4 addr show dev wlan0 | awk '{print $4}' | cut -d/ -f1 | head -1)" 192.168.123.164 --stats 60 > relay.log 2>&1 < /dev/null & echo $! > relay.pid)
    sleep 2
    kill -0 "$(cat relay.pid)" 2>/dev/null && echo "JETSON-OK lan=$LAN $(head -1 relay.log)" || { echo "RELAY-DIED"; cat relay.log; exit 1; }
    ;;
down)
    cd ~/g1_wifi 2>/dev/null && [ -f relay.pid ] && kill "$(cat relay.pid)" 2>/dev/null
    p ip route del "$ADDR/32" dev wlan0 >/dev/null
    [ -n "$LAN" ] && p ip neigh del proxy "$ADDR" dev "$LAN" >/dev/null
    p iptables -D FORWARD -s "$ADDR" -i wlan0 -j ACCEPT >/dev/null
    p iptables -D FORWARD -d "$ADDR" -o wlan0 -j ACCEPT >/dev/null
    echo "JETSON-CLEAN"
    ;;
ap)   # name and password of the access point that is up now (read-only); the password is printed on the second line
    prof=$(nmcli -t -f NAME,DEVICE connection show --active | awk -F: '$2=="wlan0"{print $1; exit}')
    [ -n "$prof" ] || { echo "NO-AP"; exit 1; }
    nmcli -g 802-11-wireless.ssid connection show "$prof"
    printf '%s\n' "$PW" | sudo -S -p '' nmcli -s -g 802-11-wireless-security.psk connection show "$prof" 2>/dev/null
    ;;
esac
EOF
}

write_cyclone() {
    mkdir -p "$(dirname "$CYCLONE")"
    cat > "$CYCLONE" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<!-- generated by tools/g1_wifi.sh: DDS over the robot's Wi-Fi. Multicast only for the discovery packet (the Jetson relays
     it to PC1), everything else unicast through the Jetson. No SocketReceiveBufferSize (Cyclone aborts if not granted). -->
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces><NetworkInterface address="$ADDR"/></Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery><Peers><Peer address="192.168.123.161"/><Peer address="$JETSON_IP"/></Peers></Discovery>
  </Domain>
</CycloneDDS>
EOF
}

active_conn() { nmcli -t -f NAME,DEVICE connection show --active | awk -F: -v d="$1" '$2==d{print $1; exit}'; }

cmd_up() {
    [ -n "${G1_PW:-}" ] || die "set G1_PW (the Jetson's password) for ssh and sudo"
    [ -n "$WLAN" ] || die "no Wi-Fi device found (G1_WLAN)"
    if [ "$(active_conn "$WLAN")" = "$PROFILE" ]; then say "already on the robot's Wi-Fi ($PROFILE), setting it up again"; fi
    ping -c1 -W2 $JETSON_IP > /dev/null 2>&1 || die "the Jetson ($JETSON_IP) does not answer: robot off, or cable/Wi-Fi not connected"

    # remember what to bring back
    if [ "$(active_conn "$WLAN")" != "$PROFILE" ]; then
        { echo "CABLE_CONN=$(active_conn "$CABLE")"; echo "LAB_CONN=$(active_conn "$WLAN")"; } > "$STATE"
    fi

    say "Jetson: route, proxy ARP, forward rules, DDS discovery relay"
    cat "$HERE/wifi_dds_relay.py" | $JETSON_SSH 'mkdir -p ~/g1_wifi && cat > ~/g1_wifi/wifi_dds_relay.py' || die "cannot copy the relay to the Jetson (ssh/password?)"
    out=$(jetson_script | remote up "$ADDR") || { echo "$out"; die "the Jetson setup failed"; }
    echo "$out" | sed 's/^/  /'
    echo "$out" | grep -q JETSON-OK || die "the Jetson setup did not confirm"

    # the AP's name changes at every boot: scan for it; password from the saved profile, else from the Jetson
    scan=$(nmcli -t -f SSID,SIGNAL device wifi list ifname "$WLAN" --rescan yes | grep '^G1-NX-AP-' | sort -t: -k2 -nr | head -1 | cut -d: -f1)
    if ! nmcli -g connection.id connection show "$PROFILE" > /dev/null 2>&1; then
        say "no saved profile $PROFILE: reading the AP's name and password from the Jetson"
        info=$(jetson_script | remote ap "$ADDR") || die "cannot read the AP data from the Jetson"
        ssid=$(echo "$info" | sed -n 1p); psk=$(echo "$info" | sed -n 2p)
        [ -n "$ssid" ] && [ -n "$psk" ] || die "the Jetson did not return the AP data (is the AP up?)"
        nmcli connection add type wifi ifname "$WLAN" con-name "$PROFILE" ssid "$ssid" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$psk" \
            connection.autoconnect no ipv4.never-default yes ipv6.method ignore > /dev/null || die "cannot create the profile"
        scan=${scan:-$ssid}
    fi
    [ -n "$scan" ] || die "no G1-NX-AP-* network in range"
    say "access point: $scan"
    if [ -n "$STATIC" ]; then
        addrs="$STATIC/24,$ADDR/32"; method=manual
    else
        addrs="$ADDR/32"; method=auto
    fi
    nmcli connection modify "$PROFILE" 802-11-wireless.ssid "$scan" ipv4.method $method ipv4.addresses "$addrs" \
        ipv4.routes "192.168.123.0/24 $AP_IP" ipv4.never-default yes connection.autoconnect no || die "cannot configure the profile"

    say "leaving the cable ($CABLE) and joining the AP: from now on no lab Wi-Fi, no internet"
    nmcli device disconnect "$CABLE" > /dev/null 2>&1
    if ! nmcli -w 45 connection up "$PROFILE" > /dev/null; then
        say "could not join the AP, going back"; cmd_down; exit 1
    fi
    sleep 2
    ok=1
    for h in $JETSON_IP 192.168.123.161; do
        if ping -I "$ADDR" -c3 -i 0.3 -W2 "$h" > /dev/null 2>&1; then say "ping $h from $ADDR: ok"; else say "ping $h from $ADDR: FAILED"; ok=0; fi
    done
    write_cyclone
    [ $ok = 1 ] || { say "the robot LAN is not reachable, going back"; cmd_down; exit 1; }
    say "up. DDS config: $CYCLONE   (tools/g1_wifi.sh env prints the exports; viz_real.sh wifi uses its own copy)"
    say "after a robot boot: G1_PW=... G1_CYCLONEDDS_URI=file://$CYCLONE tools/g1_up.sh   (LiDAR service on)"
    say "when finished: G1_PW=... tools/g1_wifi.sh down"
}

cmd_down() {
    # shellcheck disable=SC1090
    [ -f "$STATE" ] && . "$STATE"
    if ping -c1 -W2 $JETSON_IP > /dev/null 2>&1 && [ -n "${G1_PW:-}" ]; then
        say "Jetson: removing route, proxy ARP, forward rules, relay"
        jetson_script | remote down "$ADDR" | sed 's/^/  /'
    else
        say "Jetson not reachable (or no G1_PW): its runtime rules stay until its next boot, harmless"
    fi
    nmcli connection down "$PROFILE" > /dev/null 2>&1
    say "cable back"
    nmcli device connect "$CABLE" > /dev/null 2>&1
    say "lab Wi-Fi back"
    if [ -n "${LAB_CONN:-}" ]; then nmcli -w 40 connection up "$LAB_CONN" > /dev/null 2>&1; else nmcli device connect "$WLAN" > /dev/null 2>&1; fi
    # a TCP connect, not ping: many networks (the lab's included) drop ICMP to the outside
    for _ in $(seq 1 20); do timeout 3 bash -c 'exec 3<>/dev/tcp/1.1.1.1/443' > /dev/null 2>&1 && { say "internet is back"; return 0; }; sleep 2; done
    say "no internet yet (lab Wi-Fi up? check by hand)"
}

cmd_status() {
    echo "wifi device $WLAN: $(active_conn "$WLAN")   cable $CABLE: $(active_conn "$CABLE")"
    ip -br addr show "$WLAN" 2>/dev/null
    for h in $JETSON_IP 192.168.123.161 192.168.123.210 192.168.123.211; do
        printf '%s: ' "$h"; ping -c2 -W1 "$h" 2>/dev/null | tail -1 | sed 's/^rtt //' || echo "no answer"
    done
    [ -f "$CYCLONE" ] && echo "DDS config: $CYCLONE"
}

[ "${BASH_SOURCE[0]}" = "$0" ] || return 0   # sourced (tests): only define the functions
case "${1:-status}" in
up) cmd_up ;;
down) cmd_down ;;
status) cmd_status ;;
env)
    write_cyclone
    echo "export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$CYCLONE"
    ;;
*) echo "usage: $0 up|down|status|env"; exit 2 ;;
esac
