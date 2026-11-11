#!/bin/bash
# Builds librealsense 2.54.2 on the V4L2 backend (kernel uvcvideo, no RSUSB) ON the Jetson, next to the RSUSB build of
# realsense_build_254.sh, in its own build/install dirs (~/g1_realsense_254/build_lrs_v4l2, install_lrs_v4l2). Nothing else
# is touched; the realsense-ros wrapper built against the RSUSB library is NOT rebuilt (same version, same ABI: test it by
# putting install_lrs_v4l2/lib on LD_LIBRARY_PATH). No kernel patch, no root. Read the progress in
# ~/g1_realsense_254/build_v4l2.log ("BUILD EXIT 0 <date>" when it worked).
set -u
W=$HOME/g1_realsense_254
LOG=$W/build_v4l2.log
cd "$W" || exit 1
exec >> "$LOG" 2>&1
echo "== $(date '+%F %T') start (nice 19, -j3)"
mkdir -p build_lrs_v4l2 install_lrs_v4l2
cd "$W/build_lrs_v4l2" || exit 1
echo "== $(date '+%F %T') librealsense (V4L2 backend): cmake"
nice -n 19 cmake ../src/librealsense-2.54.2 \
  -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_WITH_CUDA=OFF -DBUILD_TOOLS=ON \
  -DBUILD_UNIT_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF -DCHECK_FOR_UPDATES=OFF \
  -DIMPORT_DEPTH_CAM_FW=OFF -DENABLE_CCACHE=OFF -DCMAKE_INSTALL_PREFIX="$W/install_lrs_v4l2" \
  || { echo "BUILD EXIT 1 (cmake librealsense)"; exit 1; }
echo "== $(date '+%F %T') librealsense (V4L2 backend): make"
nice -n 19 make -j3 || { echo "BUILD EXIT 2 (make librealsense)"; exit 2; }
make install || { echo "BUILD EXIT 3 (install librealsense)"; exit 3; }
echo "BUILD EXIT 0 $(date '+%F %T')"
