#!/bin/bash
# RealSense D435i ROS 2 driver on the Jetson, driven from the laptop.
#
#   tools/realsense_ros2.sh install      once: clone realsense-ros 4.0.4 here, copy it to the Jetson, build it there
#   tools/realsense_ros2.sh start [launch args]   e.g. start enable_color:=true (stop the app video first)
#   tools/realsense_ros2.sh stop | status | log
#   tools/realsense_ros2.sh stream [name=value ...]   show / change the compression (rates, JPEG quality, PNG level) of the running driver
#   tools/realsense_ros2.sh autostart on|off|status   start the driver by itself at every Jetson boot (user crontab @reboot)
#
# Needs: ssh access to unitree@192.168.123.164 (override with G1_JETSON_SSH="ssh other@host"), git, tar.
# Everything lands in /home/unitree/g1_realsense on the Jetson. See docs/06-realsense-ros2.md.
set -eu
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
JETSON_SSH=${G1_JETSON_SSH:-ssh unitree@192.168.123.164}
TAG=4.0.4
COMMIT=8abb4657c0add15f87b0edbfb67eaba2c1c2c439   # realsense-ros 4.0.4: wants librealsense 2.50.0

remote() { $JETSON_SSH bash -s -- "$@" < "$HERE/realsense_ros2_remote.sh"; }
# the remote script (also what cron runs) and the stream-encoder node (colour JPEG, depth PNG) live in ~/g1_realsense on the Jetson
push_files() {
    tar czf - -C "$HERE" realsense_ros2_remote.sh realsense_encoder_node.py \
        | $JETSON_SSH 'mkdir -p ~/g1_realsense && tar xzf - -C ~/g1_realsense && chmod +x ~/g1_realsense/realsense_ros2_remote.sh'
}

case "${1:-status}" in
install)
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    git clone --quiet --depth 1 --branch "$TAG" https://github.com/IntelRealSense/realsense-ros.git "$tmp/realsense-ros"
    [ "$(git -C "$tmp/realsense-ros" rev-parse HEAD)" = "$COMMIT" ] || { echo "tag $TAG is not commit $COMMIT, abort"; exit 1; }
    tar czf - -C "$tmp/realsense-ros" realsense2_camera realsense2_camera_msgs LICENSE \
        | $JETSON_SSH 'mkdir -p ~/g1_realsense/src && tar xzf - -C ~/g1_realsense/src'
    echo "sources copied, building on the Jetson (about 1.5 min)..."
    remote build
    ;;
start | stop | status | check | log | stream | stack)
    [ "$1" = start ] && push_files
    remote "$@"
    ;;
autostart)
    case "${2:-status}" in
    on)
        push_files
        $JETSON_SSH 'bash ~/g1_realsense/realsense_ros2_remote.sh autostart-on'
        ;;
    off | status)
        remote "autostart-$2"
        ;;
    *) echo "usage: $0 autostart on|off|status"; exit 2 ;;
    esac
    ;;
*)
    echo "usage: $0 install|start [launch args]|stop|status|check|log|stream [name=value ...]|stack [254|old]|autostart on|off|status"; exit 2
    ;;
esac
