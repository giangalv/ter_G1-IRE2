#!/bin/bash
# After every robot boot: switch the LiDAR driver on and start the RealSense ROS 2 driver, from the laptop.
#
#   G1_PW=<Jetson password> tools/g1_up.sh              # LiDAR + camera
#   G1_PW=... G1_CAMERA=false tools/g1_up.sh            # LiDAR only (keeps the phone-app video)
#
# Idempotent: a service that is already on / a driver that already runs is left alone. Nothing is
# changed permanently on the robot; the switches are lost at the next boot (that is why this exists).
# Needs: the unitree_api messages (G1_WS_SETUP, default the ter_G1U6 workspace install), DDS on the robot NIC
# (G1_NIC, default enp59s0), ssh to the Jetson (password in G1_PW, see askpass.sh). docs/04 and docs/06.
set -u
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
NIC=${G1_NIC:-enp59s0}
WS_SETUP=${G1_WS_SETUP:-$HOME/ter-home/ter_G1U6/install/setup.bash}

set +u
source /opt/ros/humble/setup.bash
[ -f "$WS_SETUP" ] && source "$WS_SETUP"
set -u
export ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"$NIC\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
python3 -c 'import unitree_api.msg' 2>/dev/null || { echo "unitree_api messages not found (source the workspace: G1_WS_SETUP=$WS_SETUP)"; exit 1; }
if [ -n "${G1_PW:-}" ]; then
    export SSH_ASKPASS="$HERE/askpass.sh" SSH_ASKPASS_REQUIRE=force DISPLAY=${DISPLAY:-:0}
    export G1_JETSON_SSH=${G1_JETSON_SSH:-ssh -o StrictHostKeyChecking=accept-new -o PreferredAuthentications=password -o PubkeyAuthentication=no unitree@192.168.123.164}
fi

echo "waiting for the robot's service manager (up to 3 min)..."
list=""
for _ in $(seq 1 18); do
    list=$(python3 "$HERE/g1_service.py" list 2>&1) && echo "$list" | grep -q "^RESPONSE" && break
    list=""; sleep 5
done
[ -n "$list" ] || { echo "robot_state did not answer: robot off, cable/NIC wrong, or still booting"; exit 1; }

# status 0 = running, 1 = stopped (checked 2026-11-05 with lidar_driver, video_hub_pc4, video_hub_pc4_chest)
if echo "$list" | grep -Eq "^  lidar_driver +status=0"; then
    echo "lidar_driver: already on"
else
    python3 "$HERE/g1_service.py" on lidar_driver | tail -2
fi

if [ "${G1_CAMERA:-true}" = "true" ]; then
    if "$HERE/realsense_ros2.sh" status 2>&1 | head -1 | grep -q "^running"; then
        echo "RealSense driver: already running"
    else
        echo "starting the RealSense driver (this stops the phone-app head video)"
        "$HERE/realsense_ros2.sh" start 2>&1 | grep -E "running|DIED|Is Up|ERROR" | cut -c1-160
    fi
fi

echo "--- check"
(cd "$(mktemp -d)" && timeout 30 python3 "$HERE/lidar_listen.py" 4 2>&1 | grep -E "msgs")
