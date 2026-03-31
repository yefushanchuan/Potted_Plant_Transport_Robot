#!/usr/bin/env bash
set -euo pipefail

base_dir="/home/qiaowen/rpp_ws/src/robot_localization/fastlio2_ros2/map/pcd"
timestamp="$(date +%Y%m%d_%H%M%S)"
out_dir="${base_dir}/${timestamp}"

mkdir -p "${out_dir}"

ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '${out_dir}', save_patches: false}"

echo "Map saved to: ${out_dir}"
