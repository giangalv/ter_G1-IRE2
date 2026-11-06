#!/bin/bash
# Palm cameras of the Inspire hands as ROS 2 images, driven from the laptop.
#
#   tools/hand_cameras_ros2.sh install      once (and after every change): copy the g1_hand_camera python package to the Jetson
#   tools/hand_cameras_ros2.sh start [node args]
#   tools/hand_cameras_ros2.sh stop | status | log
#   tools/hand_cameras_ros2.sh autostart on|off|status    start them at every Jetson boot (user crontab @reboot, no root)
#
# Needs: ssh access to unitree@192.168.123.164 (override with G1_JETSON_SSH="ssh other@host"), tar, and the
# ter_G1U6 sources (G1_HAND_CAMERA_SRC, default ~/ter-home/ter_G1U6/src/g1_hand_camera). Everything lands in
# /home/unitree/g1_hand_cameras on the Jetson. The streams reach the laptop's domain 0 through
# g1_unitree_bridge/hand_camera_relay. See docs/03-palm-cameras.md.
set -eu
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
JETSON_SSH=${G1_JETSON_SSH:-ssh unitree@192.168.123.164}
SRC=${G1_HAND_CAMERA_SRC:-$HOME/ter-home/ter_G1U6/src/g1_hand_camera}

remote() { $JETSON_SSH bash -s -- "$@" < "$HERE/hand_cameras_ros2_remote.sh"; }

case "${1:-status}" in
install)
    [ -f "$SRC/g1_hand_camera/hand_camera_node.py" ] || { echo "no g1_hand_camera sources in $SRC (set G1_HAND_CAMERA_SRC)"; exit 1; }
    tar czf - -C "$SRC" --exclude=__pycache__ g1_hand_camera \
        | $JETSON_SSH 'mkdir -p ~/g1_hand_cameras/py && rm -rf ~/g1_hand_cameras/py/g1_hand_camera && tar xzf - -C ~/g1_hand_cameras/py'
    echo "package copied to ~/g1_hand_cameras/py on the Jetson"
    ;;
start | stop | status | log)
    remote "$@"
    ;;
autostart)
    case "${2:-status}" in
    on)
        $JETSON_SSH 'mkdir -p ~/g1_hand_cameras && cat > ~/g1_hand_cameras/hand_cameras_ros2_remote.sh && chmod +x ~/g1_hand_cameras/hand_cameras_ros2_remote.sh' < "$HERE/hand_cameras_ros2_remote.sh"
        $JETSON_SSH 'bash ~/g1_hand_cameras/hand_cameras_ros2_remote.sh autostart-on'
        ;;
    off | status)
        remote "autostart-$2"
        ;;
    *) echo "usage: $0 autostart on|off|status"; exit 2 ;;
    esac
    ;;
*)
    echo "usage: $0 install|start [node args]|stop|status|log|autostart on|off|status"; exit 2
    ;;
esac
