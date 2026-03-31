#!/usr/bin/env bash
set -euo pipefail

pcd2pgm_dir="/home/qiaowen/rpp_ws/src/tools/pcd2pgm"
config_file="${pcd2pgm_dir}/config/pcd2pgm.yaml"
map_root="/home/qiaowen/rpp_ws/src/robot_localization/fastlio2_ros2/map"
pcd_root="${map_root}/pcd"
pgm_root="${map_root}/pgm"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found in PATH. Please source /opt/ros/humble/setup.bash and /home/qiaowen/rpp_ws/install/setup.bash."
  exit 1
fi

if [[ ! -f "${config_file}" ]]; then
  echo "Config file not found: ${config_file}"
  exit 1
fi

if ! ros2 pkg prefix pcd2pgm >/dev/null 2>&1; then
  echo "pcd2pgm package not found. Build and source rpp_ws before running."
  exit 1
fi

mkdir -p "${pcd_root}" "${pgm_root}"

# Set PCD_FILE here or pass a path as the first argument.
PCD_FILE="${PCD_FILE:-}"
pcd_file="${1:-${PCD_FILE}}"

if [[ -z "${pcd_file}" ]]; then
  latest_pcd="$(find "${pcd_root}" -type f -name "map.pcd" -printf "%T@ %p\n" 2>/dev/null | sort -nr | awk 'NR==1{print $2}')"
  if [[ -n "${latest_pcd}" ]]; then
    pcd_file="${latest_pcd}"
  else
    pcd_file="$(awk -F':' '/pcd_file:/ {sub(/^[ \t]+/, "", $2); gsub(/\"|'\''/, "", $2); print $2; exit}' "${config_file}")"
  fi
fi

if [[ -z "${pcd_file}" ]]; then
  echo "PCD file not set and no map.pcd found under ${pcd_root}."
  exit 1
fi

if [[ ! -f "${pcd_file}" ]]; then
  echo "PCD file not found: ${pcd_file}"
  exit 1
fi

tmp_params="$(mktemp /tmp/pcd2pgm_XXXX.yaml)"
cleanup() { rm -f "${tmp_params}"; }
trap cleanup EXIT

pcd_value="${pcd_file}"
if [[ "${pcd_value}" == *" "* ]]; then
  pcd_value="\"${pcd_value}\""
fi

awk -v pcd_value="${pcd_value}" '
  BEGIN{done=0}
  {
    if (!done && $0 ~ /pcd_file:/) {
      sub(/pcd_file:.*/, "pcd_file: " pcd_value)
      done=1
    }
    print
  }
' "${config_file}" > "${tmp_params}"

echo "Using PCD: ${pcd_file}"
echo "Params file: ${tmp_params}"

timestamp="$(date +%Y%m%d_%H%M%S)"
out_dir="${pgm_root}/${timestamp}"
mkdir -p "${out_dir}"

ros2 launch pcd2pgm pcd2pgm_launch.py params_file:="${tmp_params}" rviz_config_file:="${pcd2pgm_dir}/rviz/pcd2pgm.rviz" &
launch_pid=$!

cleanup_runtime() {
  kill "${launch_pid}" >/dev/null 2>&1 || true
}
trap cleanup_runtime EXIT

timeout 10s bash -lc "until ros2 topic info /map >/dev/null 2>&1; do sleep 0.2; done" || true

ros2 run nav2_map_server map_saver_cli -t /map -f "${out_dir}/map" || true

echo "PGM saved to: ${out_dir}"
