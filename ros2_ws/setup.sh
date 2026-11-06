#!/bin/bash
# One-time setup of the laptop-side ROS 2 workspace: checks the system packages, downloads Unitree's robot model and
# message definitions (pinned commits), builds, runs the unit tests.   Run it from anywhere:   bash ros2_ws/setup.sh
#
# Needs Ubuntu 22.04 + ROS 2 Humble (https://docs.ros.org/en/humble/Installation.html) and internet for the first run.
# Nothing here needs root: if system packages are missing it prints the apt line to run and stops.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$HERE"
ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
[ -f "$ROS_SETUP" ] || { echo "ROS 2 Humble not found ($ROS_SETUP). Install ros-humble-desktop first."; exit 1; }

need=(ros-humble-desktop ros-humble-rviz2 ros-humble-xacro ros-humble-robot-state-publisher ros-humble-tf2-ros
      ros-humble-rmw-cyclonedds-cpp python3-colcon-common-extensions python3-numpy python3-opencv python3-pytest git)
missing=()
for p in "${need[@]}"; do dpkg -s "$p" > /dev/null 2>&1 || missing+=("$p"); done
if [ ${#missing[@]} -gt 0 ]; then
    echo "Missing system packages. Run:"; echo "  sudo apt install ${missing[*]}"; exit 1
fi

echo "== 1/3 downloading Unitree's model and messages (about 120 MB the first time)"
bash src/g1_description/fetch_assets.sh
echo "== 2/3 building"
set +u; source "$ROS_SETUP"; set -u
colcon build --symlink-install --packages-up-to g1_viz g1_hand_camera unitree_hg unitree_api
echo "== 3/3 unit tests"
set +u; source install/setup.bash; set -u
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/g1_viz src/g1_unitree_bridge src/g1_action src/g1_hand_camera -q

cat <<MSG

Done. In a new terminal:
  cd $HERE && source /opt/ros/humble/setup.bash && source install/setup.bash
  bash src/g1_viz/scripts/viz_real.sh cable        # PC on the robot's Ethernet (set G1_NIC=<your NIC>)
  bash src/g1_viz/scripts/viz_real.sh wifi         # on the robot's own Wi-Fi, after:  G1_PW=<Jetson password> ../tools/g1_wifi.sh up
See ros2_ws/README.md.
MSG
