# DDS environment for talking to the G1 from this laptop (domain 0, CycloneDDS, docs/01). SOURCE it, do not run it:
#
#   source tools/g1_env.sh cable      # laptop on the robot's Ethernet (NIC from $G1_NIC, default enp59s0)
#   source tools/g1_env.sh wifi       # laptop on the robot's own Wi-Fi, after  G1_PW=... tools/g1_wifi.sh up  (docs/07)
#
# It sources ROS 2 Humble and a workspace that has the Unitree messages (unitree_api, unitree_hg): $G1_WS_INSTALL
# (a path to its setup.bash) or ros2_ws/install/setup.bash of this repo (built by  bash ros2_ws/setup.sh).
# Profiles are the ones of ros2_ws/src/g1_viz/config. It changes only the current shell. Use `ros2 ... --no-daemon`:
# the ROS 2 daemon keeps the environment of the first command it saw.
_g1_here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
_g1_mode=${1:-cable}
_g1_cfg="$_g1_here/../ros2_ws/src/g1_viz/config"
_g1_ws=${G1_WS_INSTALL:-$_g1_here/../ros2_ws/install/setup.bash}
case "$_g1_mode" in
cable)
    export G1_NIC=${G1_NIC:-enp59s0}
    _g1_xml="$_g1_cfg/cyclonedds_cable.xml" ;;
wifi)
    export G1_WIFI_ADDR=${G1_WIFI_ADDR:-192.168.123.98}
    _g1_xml="$_g1_cfg/cyclonedds_wifi.xml"
    ip -4 addr show 2>/dev/null | grep -q "inet $G1_WIFI_ADDR/" ||
        echo "warning: $G1_WIFI_ADDR is not on any interface: run  G1_PW=... tools/g1_wifi.sh up  first" ;;
*)
    echo "usage: source tools/g1_env.sh cable|wifi"; unset _g1_here _g1_mode _g1_cfg _g1_ws; return 2 2>/dev/null || exit 2 ;;
esac
[ -f /opt/ros/humble/setup.bash ] || { echo "ROS 2 Humble not found in /opt/ros/humble"; return 1 2>/dev/null || exit 1; }
[ -f "$_g1_ws" ] || { echo "no workspace with the Unitree messages at $_g1_ws (bash ros2_ws/setup.sh, or set G1_WS_INSTALL)"; return 1 2>/dev/null || exit 1; }
source /opt/ros/humble/setup.bash
source "$_g1_ws"
unset ROS_LOCALHOST_ONLY                  # a profile with ROS_LOCALHOST_ONLY=1 hides the robot
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$_g1_xml"
echo "G1 DDS environment: $_g1_mode, domain 0, $CYCLONEDDS_URI"
unset _g1_here _g1_mode _g1_cfg _g1_ws _g1_xml
