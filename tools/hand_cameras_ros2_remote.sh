#!/bin/bash
# Palm cameras of the Inspire hands as ROS 2 JPEG images, part that runs ON the Jetson as user "unitree".
# Normally called through tools/hand_cameras_ros2.sh from the laptop.
#
#   start            run g1_hand_camera's hand_camera_node (python package copied by `install` to ~/g1_hand_cameras/py)
#   stop | status | log
#
# Same reasoning as the RealSense driver (docs/06): the Jetson has ROS 2 Foxy only, and in the robot's DDS
# domain 0 Foxy's DDS dies, so the node runs in DOMAIN 42 with Fast DDS on 192.168.123.164 only; the laptop
# relay (g1_unitree_bridge/hand_camera_relay) copies the streams into domain 0. Only the JPEG stream is
# published (~30-50 kB per frame; raw 640x480 frames are 920 kB and do not fit the laptop's 416 kB UDP buffer).
# The cameras drop for 1-2 s when a D435i stream starts (docs/03); the node reopens them by itself.
set -u
WS=$HOME/g1_hand_cameras
DOMAIN=42
JETSON_IP=192.168.123.164
PIDFILE=$WS/run.pid
LOG=$WS/run.log
PARAMS="-p publish_raw:=false -p publish_compressed:=true -p publish_rate_hz:=10.0 -p jpeg_quality:=80"

env_setup() {
    set +u
    source /opt/ros/foxy/setup.bash
    set -u
    export PYTHONPATH="$WS/py:${PYTHONPATH:-}"
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=0
    export FASTRTPS_DEFAULT_PROFILES_FILE=$WS/fastdds_lan.xml
}

running() { [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; }

case "${1:-status}" in
start)
    shift
    running && { echo "already running (pid $(cat "$PIDFILE")); stop it first"; exit 0; }
    [ -d "$WS/py/g1_hand_camera" ] || { echo "not installed: run tools/hand_cameras_ros2.sh install"; exit 1; }
    cat > "$WS/fastdds_lan.xml" <<XML
<?xml version="1.0" encoding="UTF-8" ?>
<!-- Fast DDS on the robot LAN address only (the NIC name eth0/eth1 flips between boots) -->
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>g1_lan_udp</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList><address>$JETSON_IP</address></interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="g1_lan" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>g1_lan_udp</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
    env_setup
    cd "$WS" || exit 1
    # shellcheck disable=SC2086
    setsid nohup python3 -m g1_hand_camera.hand_camera_node --ros-args $PARAMS "$@" > "$LOG" 2>&1 < /dev/null &
    echo $! > "$PIDFILE"
    sleep 6
    if running; then echo "running (pid $(cat "$PIDFILE"), domain $DOMAIN)"; else echo "DIED, see $LOG"; fi
    grep -vE "^\s*$" "$LOG" | cut -c1-200 | head -8
    ;;
stop)
    if running; then
        kill -INT -- "-$(cat "$PIDFILE")" 2>/dev/null || kill -INT "$(cat "$PIDFILE")"
        for _ in 1 2 3 4 5 6 7 8; do running || break; sleep 1; done
        running && { echo "did not stop: pid $(cat "$PIDFILE")"; exit 1; }
        echo "stopped"
    else
        echo "not running"
    fi
    rm -f "$PIDFILE"
    ;;
status)
    if running; then echo "running (pid $(cat "$PIDFILE"), domain $DOMAIN)"; else echo "not running"; fi
    [ -f "$LOG" ] && grep -vE "^\s*$" "$LOG" | tail -3 | cut -c1-200
    ;;
log)
    tail -40 "$LOG" 2>/dev/null
    ;;
*)
    echo "usage: $0 start [node args]|stop|status|log"; exit 2
    ;;
esac
