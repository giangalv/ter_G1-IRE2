# Shortcuts (run from ros2_ws/):  make setup | build | test | viz_cable | viz_wifi
SHELL := /bin/bash
ROS_SETUP ?= /opt/ros/humble/setup.bash
.PHONY: setup build test viz_cable viz_wifi
setup:
	bash setup.sh
build:
	source $(ROS_SETUP) && colcon build --symlink-install --packages-up-to g1_viz g1_hand_camera unitree_hg unitree_api
test:
	source $(ROS_SETUP) && source install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/g1_viz src/g1_unitree_bridge src/g1_action src/g1_hand_camera -q
# RViz of the real robot, read-only. Cable: G1_NIC=<your NIC>. Wi-Fi: first  G1_PW=... ../tools/g1_wifi.sh up
viz_cable:
	bash src/g1_viz/scripts/viz_real.sh cable
viz_wifi:
	bash src/g1_viz/scripts/viz_real.sh wifi
