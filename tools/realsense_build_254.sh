#!/bin/bash
# Builds librealsense 2.54.2 (RSUSB backend, no kernel patch, no root) and the realsense-ros 4.54.1 wrapper ON the Jetson
# (Ubuntu 20.04, ROS 2 Foxy, L4T R35.3.1, no internet) in ~/g1_realsense_254, next to the working 2.50.0 stack in
# ~/g1_realsense. Nothing outside ~/g1_realsense_254 is touched. About 1 h at nice 19 with -j3 on the Orin NX.
#
# The Jetson has no internet: fetch the sources on the laptop and copy them over first, e.g.
#   curl -L -o librealsense-2.54.2.tar.gz https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.54.2.tar.gz
#   curl -L -o realsense-ros-4.54.1.tar.gz https://github.com/IntelRealSense/realsense-ros/archive/refs/tags/4.54.1.tar.gz
#   ssh unitree@192.168.123.164 'mkdir -p ~/g1_realsense_254/dl' && scp *.tar.gz unitree@192.168.123.164:g1_realsense_254/dl/
# then, on the Jetson (setsid nohup bash realsense_build_254.sh &), read the progress in ~/g1_realsense_254/build.log
# (the last line is "BUILD EXIT 0 <date>" when it worked). Select the stack with `tools/realsense_ros2.sh stack 254`.
# Built and used on 2026-11-06 / 2026-11-09; docs/06 ("Dropouts") says what it did and did not change.
set -u
W=$HOME/g1_realsense_254
LOG=$W/build.log
cd "$W" || exit 1
exec >> "$LOG" 2>&1
echo "== $(date '+%F %T') start (nice 19, -j3)"
mkdir -p src build_lrs install_lrs ws/src
[ -d src/librealsense-2.54.2 ] || tar -xzf dl/librealsense-2.54.2.tar.gz -C src
[ -d src/realsense-ros-4.54.1 ] || tar -xzf dl/realsense-ros-4.54.1.tar.gz -C src

echo "== $(date '+%F %T') librealsense: cmake"
cd "$W/build_lrs" || exit 1
nice -n 19 cmake ../src/librealsense-2.54.2 \
  -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_WITH_CUDA=OFF -DBUILD_TOOLS=ON \
  -DBUILD_UNIT_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF -DCHECK_FOR_UPDATES=OFF \
  -DIMPORT_DEPTH_CAM_FW=OFF -DENABLE_CCACHE=OFF -DCMAKE_INSTALL_PREFIX="$W/install_lrs" \
  || { echo "BUILD EXIT 1 (cmake librealsense)"; exit 1; }
echo "== $(date '+%F %T') librealsense: make"
nice -n 19 make -j3 || { echo "BUILD EXIT 2 (make librealsense)"; exit 2; }
make install || { echo "BUILD EXIT 3 (install librealsense)"; exit 3; }

RS2CFG=$(find "$W/install_lrs" -name realsense2Config.cmake | head -1)
[ -n "$RS2CFG" ] || { echo "BUILD EXIT 4 (realsense2Config.cmake not found)"; exit 4; }
RS2DIR=$(dirname "$RS2CFG")
echo "== $(date '+%F %T') librealsense installed, realsense2_DIR=$RS2DIR"

echo "== $(date '+%F %T') wrapper: colcon"
cd "$W/ws" || exit 1
rm -rf src/realsense2_camera src/realsense2_camera_msgs
cp -r "$W/src/realsense-ros-4.54.1/realsense2_camera" "$W/src/realsense-ros-4.54.1/realsense2_camera_msgs" src/
set +u   # Foxy's setup.bash uses unset variables
source /opt/ros/foxy/setup.bash
set -u
export MAKEFLAGS=-j3
nice -n 19 colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -Drealsense2_DIR="$RS2DIR" \
  || { echo "BUILD EXIT 5 (colcon wrapper)"; exit 5; }
echo "BUILD EXIT 0 $(date '+%F %T')"
