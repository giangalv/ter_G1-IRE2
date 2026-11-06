#!/bin/bash
# Downloads Unitree's G1 model and message definitions (pinned commits, GitHub, ~120 MB sparse
# clone) into <workspace>/third_party and fills src/g1_description/{urdf,meshes}.
#
#   bash src/g1_description/fetch_assets.sh      # from anywhere; safe to run again
#
# Then:  colcon build --symlink-install --packages-up-to g1_viz unitree_hg
#
#  * unitree_ros  -> g1_description URDFs + meshes. Mesh paths are rewritten to
#                    package://g1_description/meshes/ so RViz can find them.
#  * unitree_ros2 -> unitree_hg / unitree_go / unitree_api: the message types of the robot's DDS
#                    topics (/lowstate, ...). colcon finds them under third_party/ by itself.
set -euo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=$(cd "$HERE/../.." && pwd)
TP="$WS/third_party"

UNITREE_ROS_SHA=4005a1b0c3f464f515f380be4145cc617fbca725
UNITREE_ROS2_SHA=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
# URDFs to install; the first one is the default (G1 29 DoF + Inspire hands with force sensors,
# 17 per hand like the RH56E2-T1).
URDFS="g1_29dof_rev_1_0_with_inspire_hand_FTP g1_29dof_rev_1_0_with_inspire_hand_DFQ g1_29dof_rev_1_0"

fetch() {  # url dir sha sparse-path
    local url=$1 dir=$2 sha=$3 path=$4
    [ -d "$dir/.git" ] || git clone --quiet --depth 1 --filter=blob:none --sparse "$url" "$dir"
    git -C "$dir" sparse-checkout set "$path"
    git -C "$dir" fetch --quiet --depth 1 origin "$sha"
    git -C "$dir" checkout --quiet FETCH_HEAD
    echo "$(basename "$dir") at $(git -C "$dir" rev-parse --short HEAD)"
}

mkdir -p "$TP"
fetch https://github.com/unitreerobotics/unitree_ros.git  "$TP/unitree_ros"  "$UNITREE_ROS_SHA"  robots/g1_description
fetch https://github.com/unitreerobotics/unitree_ros2.git "$TP/unitree_ros2" "$UNITREE_ROS2_SHA" cyclonedds_ws/src/unitree

SRC="$TP/unitree_ros/robots/g1_description"
mkdir -p "$HERE/urdf" "$HERE/meshes"
for u in $URDFS; do
    sed 's#filename="meshes/#filename="package://g1_description/meshes/#g' "$SRC/$u.urdf" > "$HERE/urdf/$u.urdf"
    grep -o 'filename="package://g1_description/meshes/[^"]*"' "$HERE/urdf/$u.urdf" \
        | sed 's#.*/meshes/##;s#"$##' | sort -u | while read -r m; do cp -u "$SRC/meshes/$m" "$HERE/meshes/$m"; done
done
echo "g1_description: $(ls "$HERE/urdf" | wc -l) URDF, $(ls "$HERE/meshes" | wc -l) meshes ($(du -sh "$HERE/meshes" | cut -f1))"
