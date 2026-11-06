#!/bin/bash
# RealSense D435i ROS 2 driver, part that runs ON the Jetson as user "unitree".
# Normally called through tools/realsense_ros2.sh from the laptop:
#   ssh unitree@192.168.123.164 'bash -s -- start' < tools/realsense_ros2_remote.sh
#
#   build            colcon build of the wrapper (sources already in ~/g1_realsense/src)
#   start [args]     run the driver in the background; args are `ros2 launch` args, they win over the defaults
#   stop | status | log
#   check            are depth frames really arriving (8 s)? exit 1 if not
#   supervise        loop (what `boot` ends in): every ~30 s `check`, restart the driver after two failures; `stop` pauses it, `start` resumes
#   stream [name=value ...]   show / change the parameters of the running stream encoder (JPEG colour, PNG depth) without a restart
#   boot             what the @reboot cron entry runs: wait for the LAN address, then `start` and check that frames
#                    arrive (`check`), up to 3 attempts
#   autostart-on | autostart-off | autostart-status   add / remove / show that crontab entry (user crontab, no root)
#
# Why it looks like this (docs/06-realsense-ros2.md):
#  * the Jetson has ROS 2 Foxy and no internet. The wrapper is realsense-ros 4.0.4 (needs librealsense
#    2.50.0 = the one that ros-noetic-librealsense2 installs under /opt/ros/noetic), built in ~/g1_realsense;
#    nothing outside the home directory is touched.
#  * DOMAIN 42, Fast DDS, UDP on 192.168.123.164 only: in the robot's domain 0 Foxy's Fast DDS dies with
#    bad_alloc and Foxy's Cyclone DDS 0.7 segfaults on the robot's discovery data.
#  * the D435i cannot be shared with Unitree's videohub_pc4 (phone-app video): when the driver opened the
#    camera, videohub_pc4 died ("An error occurred!"). `start` therefore stops videohub_pc4 first (the app has no
#    video until the next boot or until `start_app_video.sh start` after `stop` here). Colour then comes from
#    the driver. The motion module (IMU) reports "Hardware Error" on this Jetson, so it is off.
#  * initial_reset:=true (hardware reset of the camera when the node starts): after a robot reboot the first run
#    lost the USB streams after ~70 s ("uvc streamer watchdog ... Device or resource busy"), the next one had
#    no colour; with the reset both streams ran for the 3 minutes watched.
set -u
WS=$HOME/g1_realsense
DOMAIN=42
JETSON_IP=192.168.123.164
PIDFILE=$WS/run.pid
LOG=$WS/run.log
CRON_TAG="# g1_realsense_autostart"
DEFAULTS="initial_reset:=true enable_color:=true rgb_camera.profile:=640x360x15 enable_depth:=true depth_module.profile:=424x240x15 enable_gyro:=false enable_accel:=false"

env_setup() {
    set +u
    source /opt/ros/foxy/setup.bash
    [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
    set -u
    export LD_LIBRARY_PATH=/opt/ros/noetic/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=0
    export FASTRTPS_DEFAULT_PROFILES_FILE=$WS/fastdds_lan.xml
}

# the pid file outlives a reboot, and a stale pid can belong to an unrelated process then: check the command line too
running() { [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null && tr '\0' ' ' < "/proc/$(cat "$PIDFILE")/cmdline" 2>/dev/null | grep -q realsense2_camera; }

ENCODER_PID=$WS/encoder.pid
encoder_running() { [ -f "$ENCODER_PID" ] && kill -0 "$(cat "$ENCODER_PID")" 2>/dev/null && tr '\0' ' ' < "/proc/$(cat "$ENCODER_PID")/cmdline" 2>/dev/null | grep -q realsense_encoder_node; }
# colour as JPEG + depth as PNG for the Wi-Fi (realsense_encoder_node.py, copied next to this script); idempotent
encoder_start() {
    encoder_running && return 0
    [ -f "$WS/realsense_encoder_node.py" ] || return 0
    env_setup
    setsid nohup python3 "$WS/realsense_encoder_node.py" > "$WS/encoder.log" 2>&1 < /dev/null &
    echo $! > "$ENCODER_PID"
}
encoder_stop() {
    if encoder_running; then
        kill -INT "$(cat "$ENCODER_PID")" 2>/dev/null; sleep 1
        encoder_running && kill -KILL "$(cat "$ENCODER_PID")" 2>/dev/null
    fi
    rm -f "$ENCODER_PID"
}

# A started driver can be alive and still dead on the USB side (2026-11-06, first start after a robot boot: "control_transfer
# returned error" from the first second on, 0 frames for an hour, process running). So count depth frames for 8 s.
streaming() {
    env_setup
    python3 - <<'PY'
import sys, time, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
rclpy.init()
node = rclpy.create_node("g1_rs_streaming_check")
count = [0]
node.create_subscription(Image, "/camera/depth/image_rect_raw", lambda m: count.__setitem__(0, count[0] + 1), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(node, timeout_sec=0.2)
print(f"depth frames in 8 s: {count[0]}")
sys.exit(0 if count[0] >= 20 else 1)
PY
}

do_stop() {
    if running; then
        kill -INT -- "-$(cat "$PIDFILE")" 2>/dev/null || kill -INT "$(cat "$PIDFILE")"
        for _ in 1 2 3 4 5 6 7 8 9 10 11 12; do running || break; sleep 1; done
        if running; then   # a driver stuck on the USB side ignores SIGINT
            echo "SIGINT ignored, killing"
            kill -KILL -- "-$(cat "$PIDFILE")" 2>/dev/null || kill -KILL "$(cat "$PIDFILE")" 2>/dev/null
            pkill -KILL -f '[r]ealsense2_camera_node' 2>/dev/null
            sleep 1
        fi
        running && { echo "did not stop: pid $(cat "$PIDFILE")"; exit 1; }
        echo "stopped"
    else
        echo "not running"
    fi
    rm -f "$PIDFILE"
    encoder_stop
}

case "${1:-status}" in
build)
    [ -d "$WS/src/realsense2_camera" ] || { echo "no sources in $WS/src (run tools/realsense_ros2.sh install from the laptop)"; exit 1; }
    env_setup
    cd "$WS" || exit 1
    MAKEFLAGS=-j3 nice -n 19 colcon build --executor sequential --packages-select realsense2_camera_msgs realsense2_camera \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -Drealsense2_DIR=/opt/ros/noetic/lib/aarch64-linux-gnu/cmake/realsense2 2>&1 | tail -6
    ;;
start)
    shift
    rm -f "$WS/user_stop"
    running && { echo "already running (pid $(cat "$PIDFILE")); stop it first"; encoder_start; exit 0; }
    [ -f "$WS/install/setup.bash" ] || { echo "not built: run tools/realsense_ros2.sh install"; exit 1; }
    # only the processes that read /dev/video4 (not videohub_pc4_chest, which reads the chest camera)
    HOLDERS='[v]ideohub_pc4 /dev/video4|[v]ideohub_pc4_patched'
    if pgrep -f "$HOLDERS" > /dev/null; then
        echo "videohub_pc4 (app video) holds the camera: stopping it"
        /unitree/sbin/mscli stopservice video_hub_pc4 > /dev/null 2>&1
        pkill -f '[v]ideohub_pc4_patched' 2>/dev/null
        for _ in 1 2 3 4 5 6; do pgrep -f "$HOLDERS" > /dev/null || break; sleep 1; done
        pgrep -f "$HOLDERS" > /dev/null && { echo "videohub_pc4 is still running, abort"; exit 1; }
    fi
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
    setsid nohup ros2 launch realsense2_camera rs_launch.py $DEFAULTS "$@" > "$LOG" 2>&1 < /dev/null &
    echo $! > "$PIDFILE"
    sleep 20   # initial_reset re-enumerates the camera first (~7 s)
    encoder_start
    if running; then echo "running (pid $(cat "$PIDFILE"), domain $DOMAIN)"; else echo "DIED, see $LOG"; fi
    grep -E "Node Is Up|Device Name|USB type|Open profile|ERROR|died" "$LOG" | grep -v hwmon | cut -c1-200 | head -8
    ;;
boot)
    export PATH="$PATH:/usr/sbin:/sbin"
    echo "$(date '+%F %T') boot: waiting for $JETSON_IP"
    for _ in $(seq 1 60); do ip -4 addr show 2>/dev/null | grep -q "inet $JETSON_IP/" && break; sleep 2; done
    sleep 20   # let master_service and the USB hub settle
    bash "$0" start
    for attempt in 1 2 3; do
        if running && streaming; then echo "$(date '+%F %T') boot: streaming (attempt $attempt)"; break; fi
        echo "$(date '+%F %T') boot: attempt $attempt not streaming, restarting"
        bash "$0" _restart
    done
    exec bash "$0" supervise
    ;;
check)
    if running && streaming; then echo "streaming"; else echo "NOT streaming"; exit 1; fi
    ;;
autostart-on)
    [ "$(basename "$0")" = realsense_ros2_remote.sh ] && [ -f "$0" ] || { echo "run it as a file (tools/realsense_ros2.sh autostart on copies it)"; exit 1; }
    LINE="@reboot /bin/bash $0 boot >> $WS/boot.log 2>&1 $CRON_TAG"
    ( crontab -l 2>/dev/null | grep -vF "$CRON_TAG"; echo "$LINE" ) | crontab -
    echo "autostart on: $LINE"
    ;;
autostart-off)
    ( crontab -l 2>/dev/null | grep -vF "$CRON_TAG" ) | crontab - 2>/dev/null || crontab -r 2>/dev/null
    [ -f "$WS/supervise.pid" ] && kill "$(cat "$WS/supervise.pid")" 2>/dev/null
    echo "autostart off"
    ;;
autostart-status)
    crontab -l 2>/dev/null | grep -F "$CRON_TAG" || echo "autostart off"
    ;;
stop)
    touch "$WS/user_stop"   # tells `supervise` that this stop is wanted
    do_stop
    ;;
_restart)
    do_stop; sleep 5; bash "$0" start
    ;;
supervise)
    # Keeps the driver streaming. 2026-11-06: the driver stayed alive with 0 frames several times (the USB side dead; cause
    # not found, docs/06). Check every ~30 s, restart after two failed checks.
    if [ -f "$WS/supervise.pid" ] && kill -0 "$(cat "$WS/supervise.pid")" 2>/dev/null \
        && tr '\0' ' ' < "/proc/$(cat "$WS/supervise.pid")/cmdline" 2>/dev/null | grep -q supervise; then
        echo "already supervising (pid $(cat "$WS/supervise.pid"))"; exit 0
    fi
    echo $$ > "$WS/supervise.pid"
    bad=0
    while true; do
        sleep 30
        [ -f "$WS/user_stop" ] && { bad=0; continue; }
        encoder_start
        if running && streaming > /dev/null 2>&1; then bad=0; continue; fi
        bad=$((bad + 1))
        [ $bad -lt 2 ] && continue
        echo "$(date '+%F %T') supervise: not streaming twice in a row (driver process $(running && echo alive || echo gone)), restarting the driver"
        # keep the log of the run that died (the restart overwrites run.log); the last 5 are kept
        cp -f "$WS/run.log" "$WS/run.before-restart.$(date +%m%d-%H%M%S).log" 2>/dev/null
        ls -t "$WS"/run.before-restart.*.log 2>/dev/null | tail -n +6 | xargs -r rm -f
        bash "$0" _restart
        bad=0
    done
    ;;
stream)   # stream [name=value ...]: no arguments = show, else set (rates in Hz, JPEG quality 1-100, PNG level 0-9)
    shift
    env_setup
    encoder_running || { echo "the stream encoder is not running (start the driver first)"; exit 1; }
    names="color_rate_hz color_quality depth_rate_hz depth_png_level"
    if [ $# -eq 0 ]; then
        for n in $names; do printf '%-16s' "$n"; ros2 param get --spin-time 3 /realsense_encoder "$n" 2>&1 | sed 's/^.*value is: *//'; done
    else
        for kv in "$@"; do
            n=${kv%%=*}; v=${kv#*=}
            case " $names " in *" $n "*) ;; *) echo "unknown parameter $n (one of: $names)"; exit 2 ;; esac
            case "$n" in *_hz) case "$v" in *.*) ;; *) v=$v.0 ;; esac ;; esac   # the rates are doubles
            ros2 param set --spin-time 3 /realsense_encoder "$n" "$v"
        done
    fi
    ;;
status)
    if running; then echo "running (pid $(cat "$PIDFILE"), domain $DOMAIN)"; else echo "not running"; fi
    [ -f "$LOG" ] && grep -vE "^\s*$|control_transfer|hwmon" "$LOG" | tail -3 | cut -c1-200
    ;;
log)
    tail -40 "$LOG" 2>/dev/null
    ;;
*)
    echo "usage: $0 build|start [launch args]|stop|status|check|supervise|log|boot|autostart-on|autostart-off|autostart-status"; exit 2
    ;;
esac
