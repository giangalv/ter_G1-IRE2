#!/bin/bash
# RViz2 of the REAL G1, from any PC. Only subscribes: nothing is sent to the robot
# (unless you set G1_BRINGUP, see below).
#
#   bash src/g1_viz/scripts/viz_real.sh cable     # PC on the robot's Ethernet (192.168.123.x)
#   bash src/g1_viz/scripts/viz_real.sh wifi      # PC on the robot's own WiFi access point: run `G1_PW=... tools/g1_wifi.sh up`
#                                                 (base repo ter_G1-IRE2) first.
#                                                 G1_WIFI_PROFILE=manip|nav|full, default manip (see below)
#
# Extra arguments go to the launch, e.g.  viz_real.sh cable use_rviz:=false
# Environment: G1_NIC=<ethernet NIC of this PC> (default enp59s0), ROS_SETUP, WS_INSTALL,
# G1_VIEW=full|hand_right|hand_left (RViz camera: whole robot, or zoomed on one hand; default full),
# G1_REALSENSE=false (no colour/depth relay from the RealSense driver),
# G1_HEAD_CAMERA=true (decode Unitree's app video instead of the RealSense colour; the two cannot run together),
# G1_HANDS=false (skip the hands: no Modbus connection, no tactile view),
# G1_HAND_TCP_TIMEOUT_S (hand Modbus timeout; default 0.02 on the cable, 0.15 on wifi),
# G1_RS_COLOR=jpeg|raw, G1_RS_DEPTH=png|raw (how the head camera streams cross the link: JPEG / lossless PNG made on the Jetson,
#   or the driver's raw images; default jpeg+png on wifi, raw on the cable; the Jetson's rates and quality are set with
#   `tools/realsense_ros2.sh stream`, docs/07-wifi.md),
# G1_HAND_2D=false (no 2D window with the flat hands; default: on when the hands are on),
# G1_HAND_CAMERAS=false (no palm-camera relay from the Jetson, see tools/hand_cameras_ros2.sh in the base repo),
# G1_LOWSTATE_TOPIC (default /lf/lowstate, 20 Hz; /lowstate is ~1 kHz and costs a full CPU core),
# G1_BRINGUP=<script> runs first: the base repo's tools/g1_up.sh switches the robot's lidar_driver service on and starts
#   the RealSense driver on the Jetson (needs G1_PW, the Jetson password; both are lost at every robot boot).
set -eu
MODE=${1:-cable}; shift || true
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=$(cd "$HERE/../../.." && pwd)

set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${WS_INSTALL:-$WS/install/setup.bash}"
set -u

# The robot's DDS is domain 0 (not the 41 / localhost-only used by the rest of the workspace).
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
SHARE=$(ros2 pkg prefix g1_viz)/share/g1_viz
case "$MODE" in
cable)
    export G1_NIC=${G1_NIC:-enp59s0}
    export CYCLONEDDS_URI="file://$SHARE/config/cyclonedds_cable.xml"
    ;;
wifi)
    # over the robot's own access point: tools/g1_wifi.sh up (base repo) must have been run first
    export G1_WIFI_ADDR=${G1_WIFI_ADDR:-192.168.123.98}
    export G1_NIC=${G1_NIC:-$(nmcli -t -f DEVICE,TYPE device 2>/dev/null | awk -F: '$2=="wifi"{print $1; exit}')}
    export CYCLONEDDS_URI="file://$SHARE/config/cyclonedds_wifi.xml"
    ip -4 addr show 2>/dev/null | grep -q "inet $G1_WIFI_ADDR/" || { echo "$G1_WIFI_ADDR is not on any interface: run  G1_PW=... tools/g1_wifi.sh up  (base repo) first"; exit 1; }
    ;;
*)
    echo "usage: $0 cable|wifi [launch args]"; exit 2
    ;;
esac

if [ -n "${G1_BRINGUP:-}" ]; then bash "$G1_BRINGUP" || echo "G1_BRINGUP failed, starting the viewer anyway"; fi

TOPIC=${G1_LOWSTATE_TOPIC:-/lf/lowstate}
# wifi only: what is shown depends on the task (the link carries ~60-100 Mbit/s; measured 2026-11-06, docs/07-wifi.md):
#   G1_WIFI_PROFILE=manip (default)  hands (tactile, 2D view, palm cameras) + head camera (JPEG colour, PNG depth), no LiDAR
#   G1_WIFI_PROFILE=nav              LiDAR cloud + head camera (JPEG colour, PNG depth), no hands (no Modbus polling)
#   G1_WIFI_PROFILE=full             everything, as on the cable (images ~4 Hz, hands time out)
# The relays only pull a stream from the Jetson while an RViz display / node is subscribed to it, so ticking or
# unticking a display in RViz also starts or stops its traffic.
SUFFIX=""
if [ "$MODE" = wifi ]; then
    case "${G1_WIFI_PROFILE:-manip}" in
    manip) SUFFIX=_manip ;;
    nav) SUFFIX=_nav; G1_HANDS=false; G1_HAND_CAMERAS=false ;;
    full) ;;
    *) echo "G1_WIFI_PROFILE must be manip, nav or full"; exit 2 ;;
    esac
fi
case "${G1_RS_COLOR:-$([ "$MODE" = wifi ] && echo jpeg || echo raw)}" in
jpeg) RS_COLOR_COMPRESSED=true ;; raw) RS_COLOR_COMPRESSED=false ;; *) echo "G1_RS_COLOR must be jpeg or raw"; exit 2 ;; esac
case "${G1_RS_DEPTH:-$([ "$MODE" = wifi ] && echo png || echo raw)}" in
png) RS_DEPTH_COMPRESSED=true ;; raw) RS_DEPTH_COMPRESSED=false ;; *) echo "G1_RS_DEPTH must be png or raw"; exit 2 ;; esac
case "${G1_VIEW:-full}" in
full) CONFIG=g1_real${SUFFIX}.rviz ;;
hand_right|hand_left) CONFIG=g1_${G1_VIEW}.rviz ;;
*) echo "G1_VIEW must be full, hand_right or hand_left"; exit 2 ;;
esac
exec ros2 launch g1_viz g1_viz.launch.py lowstate_bridge:=true lowstate_topic:=$TOPIC fixed_base:=true hands:=${G1_HANDS:-true} \
    head_camera:=${G1_HEAD_CAMERA:-false} head_video_iface:=${G1_NIC} realsense:=${G1_REALSENSE:-true} rs_color_compressed:=$RS_COLOR_COMPRESSED rs_depth_compressed:=$RS_DEPTH_COMPRESSED \
    hand_cameras:=${G1_HAND_CAMERAS:-true} hand_2d:=${G1_HAND_2D:-${G1_HANDS:-true}} \
    hand_tcp_timeout_s:=${G1_HAND_TCP_TIMEOUT_S:-$([ "$MODE" = wifi ] && echo 0.15 || echo 0.02)} \
    rviz_config:="$SHARE/config/$CONFIG" "$@"
