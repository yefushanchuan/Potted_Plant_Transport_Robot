#!/usr/bin/env bash
set -euo pipefail

# 1. 自动获取当前脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 2. 动态设置地图相关路径
map_root="${SCRIPT_DIR}/map"
pcd_root="${map_root}/pcd"
pgm_root="${map_root}/pgm"

# 3. 检查 ROS 2 环境是否 source
if ! command -v ros2 >/dev/null 2>&1; then
  echo "❌ ros2 command not found. 请确保你已经 source 了 ROS2 和当前工作空间 (e.g., source install/setup.bash)"
  exit 1
fi

# 4. 检查 pcd2pgm 功能包是否存在于工作空间中
if ! ros2 pkg prefix pcd2pgm >/dev/null 2>&1; then
  echo "❌ 找不到 pcd2pgm 功能包。请确保编译并 source 了包含该包的工作空间。"
  exit 1
fi

# 5. 动态获取 pcd2pgm 包的路径，读取默认配置和 rviz
pcd2pgm_share="$(ros2 pkg prefix pcd2pgm)/share/pcd2pgm"
config_file="${pcd2pgm_share}/config/pcd2pgm.yaml"
rviz_file="${pcd2pgm_share}/rviz/pcd2pgm.rviz"

# 容错：如果 install/share 目录下找不到，尝试在源码目录找
if [[ ! -f "${config_file}" ]]; then
  config_file="${SCRIPT_DIR}/src/tools/pcd2pgm/config/pcd2pgm.yaml"
  rviz_file="${SCRIPT_DIR}/src/tools/pcd2pgm/rviz/pcd2pgm.rviz"
fi

if [[ ! -f "${config_file}" ]]; then
  echo "❌ 找不到配置文件: ${config_file}"
  exit 1
fi

# 确保输出目录存在
mkdir -p "${pcd_root}" "${pgm_root}"

# 6. 查找最新生成的 PCD 文件 (支持命令行传参指定文件)
PCD_FILE="${PCD_FILE:-}"
pcd_file="${1:-${PCD_FILE}}"

if [[ -z "${pcd_file}" ]]; then
  # 自动寻找 pcd_root 目录下最新的 .pcd 文件 (原脚本写死了 map.pcd，这里改为寻找最新生成的任意 pcd)
  latest_pcd="$(find "${pcd_root}" -type f -name "*.pcd" -printf "%T@ %p\n" 2>/dev/null | sort -nr | awk 'NR==1{print $2}')"
  if [[ -n "${latest_pcd}" ]]; then
    pcd_file="${latest_pcd}"
  else
    # 尝试从 config 文件读取默认名字
    pcd_file="$(awk -F':' '/pcd_file:/ {sub(/^[ \t]+/, "", $2); gsub(/\"|'\''/, "", $2); print $2; exit}' "${config_file}")"
  fi
fi

if [[ -z "${pcd_file}" ]]; then
  echo "❌ 找不到任何 PCD 文件！请检查 ${pcd_root} 目录。"
  exit 1
fi

if [[ ! -f "${pcd_file}" ]]; then
  echo "❌ 指定的 PCD 文件不存在: ${pcd_file}"
  exit 1
fi

# 7. 动态修改临时配置文件（不污染原文件）
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

echo "✅ 正在转换的点云地图: ${pcd_file}"

# 8. 生成带时间戳的输出目录
timestamp="$(date +%Y%m%d_%H%M%S)"
out_dir="${pgm_root}/${timestamp}"
mkdir -p "${out_dir}"

# 9. 启动转换节点并放入后台运行
ros2 launch pcd2pgm pcd2pgm_launch.py params_file:="${tmp_params}" rviz_config_file:="${rviz_file}" &
launch_pid=$!

cleanup_runtime() {
  kill "${launch_pid}" >/dev/null 2>&1 || true
}
trap cleanup_runtime EXIT

# 10. 轮询等待 /map 话题发布（最长等待 10 秒）
echo "⏳ 等待点云被压平转换成 /map 话题..."
timeout 10s bash -lc "until ros2 topic info /map >/dev/null 2>&1; do sleep 0.2; done" || true

# 11. 调用 nav2_map_server 抓取并保存 2D 地图
echo "💾 正在生成 2D 栅格地图 (PGM/YAML)..."
ros2 run nav2_map_server map_saver_cli -t /map -f "${out_dir}/map" || true

echo "🎉 完美！2D 导航图已保存至: ${out_dir}"