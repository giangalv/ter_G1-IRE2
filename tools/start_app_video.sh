#!/bin/bash
# Phone-app video of the head camera, after every boot. Run ON the Jetson as user "unitree":
#
#   bash start_app_video.sh start     # stop the stock service, run a patched copy (until reboot)
#   bash start_app_video.sh status
#   bash start_app_video.sh stop      # stop the copy (the stock service stays stopped until the next boot)
#
# From the laptop (nothing is copied to the Jetson):
#   ssh unitree@192.168.123.164 'bash -s -- start' < tools/start_app_video.sh
#
# Why: Unitree's videohub_pc4 hard-codes the NIC name "eth0" for the app stream
# (`multicast-iface=eth0`) and its cyclonedds.xml pins "eth0". On this Jetson the robot
# LAN NIC is eth0 or eth1 depending on the boot, so after a boot the stock service runs
# but sends nothing. This script patches ONE copy of the binary in /tmp (same length, the
# original is untouched), picks the NIC that has 192.168.123.164 and selects the DDS
# interface by address. Nothing persistent is changed; a reboot undoes it.
# See docs/01-network-and-computers.md ("Phone-app video after every boot").
set -u
MSCLI=/unitree/sbin/mscli
SRC=/unitree/module/video_hub_pc4/videohub_pc4
DST=/tmp/videohub_pc4_patched
LOG=/tmp/videohub_pc4_manual.log
JETSON_IP=192.168.123.164

copy_pids() { pgrep -f '[v]ideohub_pc4_patched'; }
stock_pids() { pgrep -f '[v]ideohub_pc4 /dev/video4'; }

stop_copy() {
    for _ in 1 2 3 4 5 6; do
        pids=$(copy_pids) || return 0
        kill $pids 2>/dev/null
        sleep 1
    done
    echo "the patched copy did not stop: $(copy_pids)"; return 1
}

case "${1:-start}" in
status)
    echo "patched copy: $(copy_pids | tr '\n' ' ')"
    echo "stock service: $(stock_pids | tr '\n' ' ')"
    tail -2 "$LOG" 2>/dev/null
    ;;
stop)
    stop_copy || exit 1
    rm -f "$DST"
    echo "copy stopped. The stock service stays stopped: on this Jetson it pins eth0 and sends nothing,"
    echo "and the master would only retry it in a loop. A reboot, or '$MSCLI startservice video_hub_pc4', brings it back."
    ;;
start)
    if copy_pids >/dev/null; then echo "the patched copy is already running: $(copy_pids | tr '\n' ' ')"; exit 0; fi
    NIC=$(ip -br -4 addr | awk -v ip="$JETSON_IP/" 'index($3, ip) == 1 {print $1}')
    [ -n "$NIC" ] || { echo "no interface has $JETSON_IP, abort"; exit 1; }
    [ ${#NIC} -eq 4 ] || { echo "NIC name '$NIC' is not 4 characters long, cannot patch in place"; exit 1; }
    [ -e /dev/video4 ] || { echo "/dev/video4 is missing (is the D435i plugged in? see docs/01), abort"; exit 1; }
    # the stock service holds the camera: ask the master to stop it (it does not respawn a stopped one)
    "$MSCLI" stopservice video_hub_pc4
    for _ in $(seq 1 10); do stock_pids >/dev/null || break; sleep 1; done
    stock_pids >/dev/null && { echo "the stock videohub_pc4 is still running, abort"; exit 1; }
    python3 - "$SRC" "$DST" "$NIC" <<'PY' || exit 1
import sys
src, dst, nic = sys.argv[1:4]
data = open(src, "rb").read()
old = b"multicast-iface=eth0"
assert data.count(old) == 1, f"expected one occurrence, found {data.count(old)}"
open(dst, "wb").write(data.replace(old, b"multicast-iface=" + nic.encode()))
print(f"patched copy written to {dst} (multicast-iface={nic}); the original is untouched")
PY
    chmod +x "$DST"
    export CYCLONEDDS_URI="<CycloneDDS><Domain id=\"any\"><General><Interfaces><NetworkInterface address=\"$JETSON_IP\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
    cd /unitree/module/video_hub_pc4 || exit 1
    setsid nohup "$DST" /dev/video4 > "$LOG" 2>&1 < /dev/null &
    sleep 4
    copy_pids >/dev/null && { echo "running: $(copy_pids | tr '\n' ' ')  (log $LOG)"; tail -1 "$LOG"; } || { echo "NOT RUNNING, see $LOG"; tail -5 "$LOG"; exit 1; }
    ;;
*)
    echo "usage: $0 start|stop|status"; exit 2
    ;;
esac
