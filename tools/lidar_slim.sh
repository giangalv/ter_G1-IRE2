#!/bin/bash
# Lighter LiDAR cloud for the robot's Wi-Fi: a filter node on the Jetson (tools/lidar_slim_node.py, docs/07-wifi.md).
#
#   G1_PW=<Jetson password> tools/lidar_slim.sh start [node args]   # e.g. --stride 2, --head-range 0.8
#   tools/lidar_slim.sh stop | status | log
#
# The node subscribes the robot's /utlidar/cloud_livox_mid360 over the robot LAN (cable inside the robot, DDS domain 0,
# Foxy + ~/cyclonedds_ws, its own Cyclone config pinned to the LAN address) and publishes /g1/lidar/slim: the same cloud
# without the empty points (x=y=z=0) and without the head's own returns. The laptop then reads /g1/lidar/slim
# (G1_LIDAR_TOPIC=/g1/lidar/slim for viz_real.sh) and the full cloud never crosses the Wi-Fi.
# Everything lives in ~/g1_lidar_slim on the Jetson, runs as the normal user, and is gone (the process) at the next boot.
set -u
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
JETSON_IP=192.168.123.164
if [ -n "${G1_PW:-}" ]; then
    export SSH_ASKPASS="$HERE/askpass.sh" SSH_ASKPASS_REQUIRE=force DISPLAY=${DISPLAY:-:0}
fi
JETSON_SSH=${G1_JETSON_SSH:-ssh -o StrictHostKeyChecking=accept-new -o PreferredAuthentications=password -o PubkeyAuthentication=no -o ConnectTimeout=8 unitree@$JETSON_IP}

remote_script() {   # runs on the Jetson: $1 = start | stop | status | log, then the node's own arguments
    cat <<'EOF'
D=$HOME/g1_lidar_slim
ACTION=$1; shift
running() { [ -f "$D/slim.pid" ] && [ -r "/proc/$(cat "$D/slim.pid")/cmdline" ] && tr '\0' ' ' < "/proc/$(cat "$D/slim.pid")/cmdline" | grep -q lidar_slim_node; }
case "$ACTION" in
start)
    running && { echo "already running (pid $(cat "$D/slim.pid"))"; exit 0; }
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
    cat > "$D/run.sh" <<'RUN'
#!/bin/bash
set +u
source /opt/ros/foxy/setup.bash
source "$HOME/cyclonedds_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 CYCLONEDDS_URI=file://$HOME/g1_lidar_slim/cyclonedds.xml
exec python3 "$HOME/g1_lidar_slim/lidar_slim_node.py" "$@"
RUN
    chmod +x "$D/run.sh"
    cd "$D" || exit 1
    rm -f "$HOME/slim.pid"
    (setsid nohup ./run.sh "$@" > "$D/slim.log" 2>&1 < /dev/null & echo $! > "$D/slim.pid")
    sleep 8
    running && { echo "started (pid $(cat "$D/slim.pid"))"; tail -3 "$D/slim.log"; } || { echo "DIED"; cat "$D/slim.log"; exit 1; }
    ;;
stop)
    running && { kill "$(cat "$D/slim.pid")"; sleep 1; echo "stopped"; } || echo "not running"
    ;;
status)
    running && { echo "running (pid $(cat "$D/slim.pid"))"; tail -2 "$D/slim.log"; } || echo "not running"
    ;;
log)
    tail -20 "$D/slim.log"
    ;;
esac
EOF
}

case "${1:-status}" in
start)
    shift
    $JETSON_SSH 'mkdir -p ~/g1_lidar_slim && cat > ~/g1_lidar_slim/lidar_slim_node.py' < "$HERE/lidar_slim_node.py" || { echo "cannot copy the node to the Jetson"; exit 1; }
    remote_script | $JETSON_SSH 'bash -s -- start '"$*"
    ;;
stop | status | log)
    remote_script | $JETSON_SSH 'bash -s -- '"$1"
    ;;
*)
    echo "usage: $0 start [node args]|stop|status|log"; exit 2
    ;;
esac
