#!/bin/bash
# RealSense D435i ROS 2 driver, part that runs ON the Jetson as user "unitree".
# Normally called through tools/realsense_ros2.sh from the laptop:
#   ssh unitree@192.168.123.164 'bash -s -- start' < tools/realsense_ros2_remote.sh
#
#   build            colcon build of the wrapper (sources already in ~/g1_realsense/src)
#   start [args]     run the driver in the background; args are `ros2 launch` args, they win over the defaults
#   stop | status | log
#   check            are depth frames really arriving (8 s)? exit 1 if not
#   supervise        loop (what `boot` ends in): every 2 s look for the driver's "camera lost" line and restart at once (camera back in ~20 s);
#                    every ~30 s `check`, restart the driver after two failures; `stop` pauses it, `start` resumes
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

# Which RealSense stack the driver runs from. Without the file below: librealsense 2.50.0 (apt, ros-noetic-librealsense2) and
# realsense-ros 4.0.4 in $WS. With "254" in it: librealsense 2.54.2 and realsense-ros 4.54.1 built in $W254 (side by side,
# nothing of the old stack is touched). Set with `stack 254` / `stack old`, takes effect at the next driver start.
STACK_FILE=$WS/stack
W254=$HOME/g1_realsense_254
new_stack() { [ "$(cat "$STACK_FILE" 2>/dev/null)" = 254 ] && [ -f "$W254/ws/install/setup.bash" ]; }

env_setup() {
    set +u
    source /opt/ros/foxy/setup.bash
    if new_stack; then
        source "$W254/ws/install/setup.bash"
        # "v4l2" in $WS/lrs_backend: librealsense 2.54.2 on the kernel V4L2 backend (install_lrs_v4l2, built by
        # realsense_build_254_v4l2.sh) instead of RSUSB; the wrapper is not rebuilt, LD_LIBRARY_PATH picks the library
        LRS=install_lrs
        [ "$(cat "$WS/lrs_backend" 2>/dev/null)" = v4l2 ] && [ -d "$W254/install_lrs_v4l2" ] && LRS=install_lrs_v4l2
        export LD_LIBRARY_PATH=$(dirname "$(find "$W254/$LRS" -name 'librealsense2.so' | head -1)"):${LD_LIBRARY_PATH:-}
    else
        [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
        export LD_LIBRARY_PATH=/opt/ros/noetic/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}
    fi
    set -u
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

# The driver says it lost the camera in its own log: "uvc streamer watchdog triggered" (+ EBUSY on every endpoint) within a second of the
# kernel's "usb 2-2.3: reset SuperSpeed Gen 1 USB device". Seen in all 4 saved logs of runs that died after "RealSense Node Is Up"
# (2026-11-06, 2026-11-09), never before that line (initial_reset makes the camera re-enumerate during the start), never in a healthy run.
# Much faster than counting frames (a check takes 8 s and two failed ones are needed): the supervisor looks for it every 2 s.
# The V4L2 backend (lrs_backend=v4l2, camera on USB 2.0) has its own: after an isochronous error in the kernel ("uvcvideo: Non-zero status
# (-75) in video completion handler", every 10-40 min) one stream can stay silent while the other goes on (2026-11-12: colour dead, depth
# 15 Hz, the process alive); the driver then repeats "Frames didn't arrived within 5 seconds" every 5 s.
dead_signature() { [ -f "$LOG" ] && awk '/RealSense Node Is Up/{up=1} up && /uvc streamer watchdog triggered|Frames didn.t arrived within 5 seconds/{found=1; exit} END{exit !found}' "$LOG"; }

# A started driver can be alive and still dead on the USB side (2026-11-06, first start after a robot boot: "control_transfer
# returned error" from the first second on, 0 frames for an hour, process running). So count the depth frames for 8 s, and the colour
# ones too when the driver publishes them (one stream can die alone, see dead_signature). The 640x480 images need big UDP receive
# buffers (net.core.rmem_max/rmem_default 25 MB, docs/06), else this check misses frames that the camera did produce.
streaming() {
    env_setup
    python3 - <<'PY'
import sys, time, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
rclpy.init()
node = rclpy.create_node("g1_rs_streaming_check")
topics = {"depth": "/camera/depth/image_rect_raw", "color": "/camera/color/image_raw"}
count = {k: 0 for k in topics}
def counter(k):
    return lambda m: count.__setitem__(k, count[k] + 1)
for k, t in topics.items():
    node.create_subscription(Image, t, counter(k), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(node, timeout_sec=0.2)
ok = count["depth"] >= 20
if node.count_publishers(topics["color"]) > 0:   # colour disabled in the launch args: no publisher, not checked
    ok = ok and count["color"] >= 20
print(f"frames in 8 s: depth {count['depth']}, color {count['color']}")
sys.exit(0 if ok else 1)
PY
}

do_stop() {
    if running; then
        # FAST_KILL=1 (the supervisor, camera already lost): skip the 12 s wait. The `ros2 launch` process, started with & from a
        # non-interactive shell, ignores SIGINT (SigIgn mask checked 2026-11-09), so that wait always ended in SIGKILL anyway; the
        # camera node below it does get the SIGINT and shuts the stream down, which is pointless once the camera is gone.
        if [ -z "${FAST_KILL:-}" ]; then
            kill -INT -- "-$(cat "$PIDFILE")" 2>/dev/null || kill -INT "$(cat "$PIDFILE")"
            for _ in 1 2 3 4 5 6 7 8 9 10 11 12; do running || break; sleep 1; done
        fi
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
    # $WS/extra_args (optional, one line of `name:=value` launch args) is added to every start, also the supervisor's restarts,
    # which pass no args of their own. For experiments: delete the file to go back to the defaults.
    EXTRA=$(cat "$WS/extra_args" 2>/dev/null || true)
    # shellcheck disable=SC2086
    setsid nohup ros2 launch realsense2_camera rs_launch.py $DEFAULTS $EXTRA "$@" > "$LOG" 2>&1 < /dev/null &
    echo $! > "$PIDFILE"
    # initial_reset re-enumerates the camera first (~7 s); the node is up ~13 s after the launch. Up to 24 s: a camera dead from the start never says it.
    for _ in $(seq 1 48); do
        running || break
        grep -q "RealSense Node Is Up" "$LOG" 2>/dev/null && break
        sleep 0.5
    done
    sleep 1
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
    # RESTART_PAUSE: seconds between the end of the old driver and the start of the new one. 8 s was tried on 2026-11-09 (first restart
    # after a camera loss stalling at "Resetting device...": 1 of 2 failed with 8 s, 4 of 5 with 2 s): no clear gain, back to 2 s (docs/06).
    do_stop; sleep "${RESTART_PAUSE:-2}"; bash "$0" start
    ;;
supervise)
    # Keeps the driver streaming. 2026-11-06: the driver stayed alive with 0 frames several times (the USB side dead; cause
    # not found, docs/06). Check every ~30 s, restart after two failed checks.
    if [ -f "$WS/supervise.pid" ] && kill -0 "$(cat "$WS/supervise.pid")" 2>/dev/null \
        && tr '\0' ' ' < "/proc/$(cat "$WS/supervise.pid")/cmdline" 2>/dev/null | grep -q supervise; then
        echo "already supervising (pid $(cat "$WS/supervise.pid"))"; exit 0
    fi
    echo $$ > "$WS/supervise.pid"
    # Two ways to notice a dead camera: the driver's own "streamer watchdog" line (looked for every 2 s, restart at once, no SIGINT wait:
    # the camera is back ~20 s after it died, was ~85 s) and, as a fallback for a driver that is alive with 0 frames without that line
    # (dead from the start, 2026-11-06), a frame count every ~30 s, restart after two failures.
    bad=0; tick=0
    save_log_and_restart() {
        # A restart can fail by itself (2026-11-09: after the node's own "Resetting device..." it never found the camera again,
        # although the kernel had re-enumerated it; at the next try it came up): when "Node Is Up" does not show within the start's
        # 24 s, try again at once, up to 3 times, instead of waiting for two failed frame checks (~100 s).
        for attempt in 1 2 3; do
            # keep the log of the run that died (the restart overwrites run.log); the last 5 are kept
            cp -f "$WS/run.log" "$WS/run.before-restart.$(date +%m%d-%H%M%S).log" 2>/dev/null
            ls -t "$WS"/run.before-restart.*.log 2>/dev/null | tail -n +6 | xargs -r rm -f
            if [ "$attempt" = 1 ]; then FAST_KILL=${1:-} bash "$0" _restart; else FAST_KILL=1 bash "$0" _restart; fi
            grep -q "RealSense Node Is Up" "$LOG" 2>/dev/null && break
            echo "$(date '+%F %T') supervise: attempt $attempt, the node did not come up"
        done
        bad=0; tick=0
    }
    while true; do
        sleep 2
        [ -f "$WS/user_stop" ] && { bad=0; tick=0; continue; }
        if running && dead_signature; then
            echo "$(date '+%F %T') supervise: the driver reports the camera lost (uvc streamer watchdog or frames timeout), restarting the driver"
            save_log_and_restart 1
            continue
        fi
        tick=$((tick + 1))
        [ $tick -lt 15 ] && continue
        tick=0
        encoder_start
        if running && streaming > /dev/null 2>&1; then bad=0; continue; fi
        bad=$((bad + 1))
        [ $bad -lt 2 ] && continue
        echo "$(date '+%F %T') supervise: not streaming twice in a row (driver process $(running && echo alive || echo gone)), restarting the driver"
        save_log_and_restart
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
stack)    # stack [254|old]: show / choose the librealsense + wrapper pair (see STACK_FILE); restart the driver to apply
    case "${2:-}" in
    254) [ -f "$W254/ws/install/setup.bash" ] || { echo "the 2.54.2 stack is not built in $W254"; exit 1; }
         echo 254 > "$STACK_FILE" ;;
    old) rm -f "$STACK_FILE" ;;
    "") ;;
    *) echo "usage: $0 stack [254|old]"; exit 2 ;;
    esac
    if new_stack; then echo "stack: librealsense 2.54.2 + realsense-ros 4.54.1 ($W254)"; else echo "stack: librealsense 2.50.0 + realsense-ros 4.0.4 ($WS)"; fi
    ;;
status)
    if running; then echo "running (pid $(cat "$PIDFILE"), domain $DOMAIN)"; else echo "not running"; fi
    [ -f "$LOG" ] && grep -vE "^\s*$|control_transfer|hwmon" "$LOG" | tail -3 | cut -c1-200
    ;;
log)
    tail -40 "$LOG" 2>/dev/null
    ;;
*)
    echo "usage: $0 build|start [launch args]|stop|status|check|supervise|log|stack [254|old]|boot|autostart-on|autostart-off|autostart-status"; exit 2
    ;;
esac
