#!/bin/bash
# Record a rosbag ON the Jetson (no cable, no Wi-Fi needed while the robot walks), then copy it out afterwards.
#
#   G1_PW=<Jetson password> [G1_NOTE='what this walk is'] tools/lab_record.sh start [NAME]   # NAME default "lab"; ~/g1_bags/NAME-<time>/
#   G1_PW=... tools/lab_record.sh status | stop | list
#   G1_PW=... tools/lab_record.sh pull NAME-<time> [DEST_DIR]      # needs the cable (about 90 MB/s); DEST default ./g1_bags
#   G1_PW=... tools/lab_record.sh rm NAME-<time>                   # asks first
#
# Two bags per recording, because the robot has two DDS worlds (checked 2026-11-18, see docs/04 and docs/06):
#   <dir>/lidar   DDS domain 0 (Foxy + ~/cyclonedds_ws, like lidar_slim.sh): /utlidar/cloud_livox_mid360 (~10 Hz, 4.4 MB/s,
#                 per-point `time` ns offset in the scan, `ring` 0..3, `intensity`), /utlidar/imu_livox_mid360 (200 Hz, linear
#                 acceleration in g, NOT m/s^2), /state_estimator/odom_pelvis (~50 Hz)
#   <dir>/camera  DDS domain 42 (Fast DDS, the RealSense driver's world): D435i colour + depth + camera_info. Default RAW 15 Hz
#                 (~23 MB/s, ~80 GB/hour: the Jetson has 1.8 TB); G1_BAG_RAW=false records the compressed pair that the
#                 encoder node makes for the Wi-Fi (JPEG colour + lossless PNG depth, much smaller but only a few Hz)
# Both are stamped by the robot's clock (PC1 and the Jetson agree to < 1 ms; both run ~56 s behind the laptop, see
# g1-rviz notes), so the two bags line up by the header stamps or by the receive times in the bags.
# Read with `ros2 bag info` / rosbag2 on any Humble machine (sqlite3); a bag whose recorder was killed has no metadata.yaml:
# `ros2 bag reindex` (Humble+) rebuilds it.
#
# Needs: lidar_driver ON (tools/g1_up.sh or `g1_service.py on lidar_driver`; it is off after every boot) and the RealSense driver
# up (autostart). The robot is NOT touched: the recorders only subscribe. Files live in ~/g1_bags on the Jetson (user level, no root).
# Refuses to start with less than G1_BAG_MIN_FREE_GB (default 60) free.
set -u
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
JETSON_IP=${G1_JETSON_IP:-192.168.123.164}   # 10.42.0.1 = the Jetson on its own Wi-Fi AP (no relay needed for ssh)
if [ -n "${G1_PW:-}" ]; then
    export SSH_ASKPASS="$HERE/askpass.sh" SSH_ASKPASS_REQUIRE=force DISPLAY=${DISPLAY:-:0}
fi
SSH_OPTS="-o StrictHostKeyChecking=accept-new -o PreferredAuthentications=password -o PubkeyAuthentication=no -o ConnectTimeout=8"
JETSON_SSH=${G1_JETSON_SSH:-ssh $SSH_OPTS unitree@$JETSON_IP}
RAW=${G1_BAG_RAW:-true}
MIN_FREE=${G1_BAG_MIN_FREE_GB:-60}

remote_script() {   # runs on the Jetson: $1 = start|stop|status|list|rm, then its arguments
    cat <<'EOF'
D=$HOME/g1_bags
mkdir -p "$D"
ACTION=$1; shift
alive() { [ -f "$D/$1.pid" ] && [ -r "/proc/$(cat "$D/$1.pid")/cmdline" ] && tr '\0' ' ' < "/proc/$(cat "$D/$1.pid")/cmdline" | grep -q "ros2.*bag.*record"; }
case "$ACTION" in
start)
    NAME=$1; RAW=$2; MIN_FREE=$3; NOTE=$4
    alive rec_lidar || alive rec_cam && { echo "already recording: $(cat "$D/current" 2>/dev/null)"; exit 1; }
    FREE=$(df -BG --output=avail "$D" | tail -1 | tr -dc 0-9)
    [ "$FREE" -ge "$MIN_FREE" ] || { echo "only ${FREE} GB free (< $MIN_FREE): free some space first"; exit 1; }
    OUT="$D/$NAME-$(date +%Y%m%d-%H%M%S)"; mkdir -p "$OUT"
    cat > "$D/cyclonedds.xml" <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces><NetworkInterface address="192.168.123.164"/></Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
XML
    cat > "$D/run_lidar.sh" <<'RUN'
#!/bin/bash
set +u
source /opt/ros/foxy/setup.bash
source "$HOME/cyclonedds_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 CYCLONEDDS_URI=file://$HOME/g1_bags/cyclonedds.xml
exec ros2 bag record -o "$1/lidar" /utlidar/cloud_livox_mid360 /utlidar/imu_livox_mid360 /state_estimator/odom_pelvis
RUN
    cat > "$D/run_cam.sh" <<'RUN'
#!/bin/bash
set +u
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/g1_realsense/fastdds_lan.xml
if [ "$2" = true ]; then T="/camera/color/image_raw /camera/depth/image_rect_raw"
else T="/camera/color/image_raw/compressed /camera/depth/image_rect_raw/compressedDepth"; fi
exec ros2 bag record -o "$1/camera" $T /camera/color/camera_info /camera/depth/camera_info
RUN
    chmod +x "$D/run_lidar.sh" "$D/run_cam.sh"
    { echo "started (robot clock) $(date +%s.%N)"; echo "raw_camera $RAW"; echo "note $NOTE"; } > "$OUT/session.txt"
    echo "$OUT" > "$D/current"
    (setsid nohup "$D/run_lidar.sh" "$OUT" > "$OUT/lidar.log" 2>&1 < /dev/null & echo $! > "$D/rec_lidar.pid")
    (setsid nohup "$D/run_cam.sh" "$OUT" "$RAW" > "$OUT/camera.log" 2>&1 < /dev/null & echo $! > "$D/rec_cam.pid")
    sleep 9
    for w in lidar camera; do
        n=$(grep -c "Subscribed to topic" "$OUT/$w.log"); echo "$w: subscribed to $n topics"
    done
    alive rec_lidar && alive rec_cam && echo "recording into $OUT" || { echo "A RECORDER DIED:"; tail -n 5 "$OUT"/*.log; exit 1; }
    grep -q "utlidar/cloud" "$OUT/lidar.log" || echo "WARNING: no LiDAR cloud topic yet: is lidar_driver on? (g1_service.py on lidar_driver)"
    ;;
stop)
    for p in rec_lidar rec_cam; do
        if alive $p; then kill -INT "$(cat "$D/$p.pid")"; fi
    done
    for i in 1 2 3 4 5 6 7 8 9 10 11 12; do alive rec_lidar || alive rec_cam || break; sleep 1; done
    for p in rec_lidar rec_cam; do
        alive $p && { echo "$p did not stop with INT, TERM"; kill -TERM "$(cat "$D/$p.pid")"; sleep 3; }
        alive $p && { echo "$p still alive, KILL (the bag will need ros2 bag reindex)"; kill -9 "$(cat "$D/$p.pid")"; }
        rm -f "$D/$p.pid"
    done
    OUT=$(cat "$D/current" 2>/dev/null)
    source /opt/ros/foxy/setup.bash
    for w in lidar camera; do
        echo "== $OUT/$w"; [ -f "$OUT/$w/metadata.yaml" ] && ros2 bag info "$OUT/$w" 2>&1 | grep -E "Duration|Bag size|Count" | sed 's/ *| Serialization.*//' || echo "no metadata.yaml"
    done
    du -sh "$OUT"
    ;;
status)
    for p in rec_lidar rec_cam; do alive $p && echo "$p: recording (pid $(cat "$D/$p.pid"))" || echo "$p: not running"; done
    OUT=$(cat "$D/current" 2>/dev/null); [ -d "$OUT" ] && { echo "current: $OUT ($(du -sh "$OUT" | cut -f1), $(head -1 "$OUT/session.txt"))"; }
    df -h --output=avail "$D" | tail -1 | sed 's/^/free: /'
    ;;
list)
    du -sh "$D"/*-????????-?????? 2>/dev/null; df -h --output=avail "$D" | tail -1 | sed 's/^/free: /'
    ;;
rm)
    [ -n "$1" ] && [ -d "$D/$1" ] || { echo "no such bag dir $D/$1"; exit 1; }
    alive rec_lidar || alive rec_cam && { echo "a recording is running"; exit 1; }
    rm -rf -- "$D/$1" && echo "removed $D/$1"
    ;;
esac
EOF
}

case "${1:-status}" in
start)
    remote_script | $JETSON_SSH "bash -s -- start ${2:-lab} $RAW $MIN_FREE '${G1_NOTE:-}'"
    ;;
stop | status | list)
    remote_script | $JETSON_SSH "bash -s -- $1"
    ;;
rm)
    [ -n "${2:-}" ] || { echo "usage: $0 rm NAME-<time>"; exit 2; }
    read -r -p "delete $2 from the Jetson? [y/N] " a; [ "$a" = y ] || exit 0
    remote_script | $JETSON_SSH "bash -s -- rm $2"
    ;;
pull)
    [ -n "${2:-}" ] || { echo "usage: $0 pull NAME-<time> [DEST_DIR]"; exit 2; }
    DEST=${3:-./g1_bags}; mkdir -p "$DEST"
    if command -v rsync > /dev/null && $JETSON_SSH 'command -v rsync' > /dev/null 2>&1; then
        rsync -a --partial $([ -t 1 ] && echo --info=progress2) -e "ssh $SSH_OPTS" "unitree@$JETSON_IP:g1_bags/$2" "$DEST/"
    else
        scp -r $SSH_OPTS "unitree@$JETSON_IP:g1_bags/$2" "$DEST/"
    fi && echo "copied to $DEST/$2 ($(du -sh "$DEST/$2" | cut -f1)); read it with ros2 bag info $DEST/$2/lidar"
    ;;
*)
    echo "usage: $0 start [NAME]|stop|status|list|pull NAME [DEST]|rm NAME"; exit 2
    ;;
esac
