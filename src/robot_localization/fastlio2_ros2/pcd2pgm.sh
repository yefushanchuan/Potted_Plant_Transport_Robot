#!/usr/bin/env bash

# ==========================================
# 1. 动态获取路径
# ==========================================
# 获取当前脚本所在的绝对路径 (.../src/robot_localization/fastlio2_ros2)
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 动态计算工作空间根目录 (向上退 3 级目录到达 Potted_Plant_Transport_Robot)
ws_dir="$( cd "${script_dir}/../../.." &> /dev/null && pwd )"


# ==========================================
# 2. 自动 Source 环境变量 (必须放在 set -e 之前)
# ==========================================
# 加载 ROS 2 底层环境 (如果是其他版本，比如 foxy/iron，只需改 humble 即可)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# 动态加载当前工作空间的 setup.bash
if [ -f "${ws_dir}/install/setup.bash" ]; then
    source "${ws_dir}/install/setup.bash"
else
    echo "❌ 错误: 未找到 ${ws_dir}/install/setup.bash，请确保你已经 colcon build 编译过了！"
    exit 1
fi


# ==========================================
# 3. 开启严格模式
# ==========================================
# 放在 source 后面，防止 ROS 2 自带脚本触发未绑定变量的报错退出
set -euo pipefail


# ==========================================
# 4. 初始化地图相关的目录
# ==========================================
map_root="${script_dir}/map"
pcd_root="${map_root}/pcd"
pgm_root="${map_root}/pgm"

mkdir -p "${pcd_root}" "${pgm_root}"

# 检查 pcd2pgm 功能包是否存在于环境变量中
if ! ros2 pkg prefix pcd2pgm >/dev/null 2>&1; then
  echo "❌ 找不到 pcd2pgm 功能包。请确保编译了该工具包。"
  exit 1
fi

# 动态获取 pcd2pgm 包的配置文件路径
pcd2pgm_share="$(ros2 pkg prefix pcd2pgm)/share/pcd2pgm"
config_file="${pcd2pgm_share}/config/pcd2pgm.yaml"
rviz_file="${pcd2pgm_share}/rviz/pcd2pgm.rviz"

# 容错：如果 install 目录下找不到，尝试在源码目录找
if [[ ! -f "${config_file}" ]]; then
  config_file="${ws_dir}/src/tools/pcd2pgm/config/pcd2pgm.yaml"
  rviz_file="${ws_dir}/src/tools/pcd2pgm/rviz/pcd2pgm.rviz"
fi

if [[ ! -f "${config_file}" ]]; then
  echo "❌ 找不到配置文件: ${config_file}"
  exit 1
fi


# ==========================================
# 5. 查找最新生成的 PCD 文件
# ==========================================
PCD_FILE="${PCD_FILE:-}"
pcd_file="${1:-${PCD_FILE}}"

if [[ -z "${pcd_file}" ]]; then
  # 自动寻找 pcd_root 目录下最新的 .pcd 文件
  latest_pcd="$(find "${pcd_root}" -type f -name "*.pcd" -printf "%T@ %p\n" 2>/dev/null | sort -nr | awk 'NR==1{print $2}')"
  if [[ -n "${latest_pcd}" ]]; then
    pcd_file="${latest_pcd}"
  else
    echo "❌ 找不到任何 PCD 文件！请检查 ${pcd_root} 目录。"
    exit 1
  fi
fi

if [[ ! -f "${pcd_file}" ]]; then
  echo "❌ 指定的 PCD 文件不存在: ${pcd_file}"
  exit 1
fi


# ==========================================
# 6. 动态修改配置并启动转换节点
# ==========================================
# 生成临时配置文件，防止污染原文件
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

# 生成带时间戳的输出目录
timestamp="$(date +%Y%m%d_%H%M%S)"
out_dir="${pgm_root}/${timestamp}"
mkdir -p "${out_dir}"

# 启动转换节点并放入后台运行
ros2 launch pcd2pgm pcd2pgm_launch.py params_file:="${tmp_params}" rviz_config_file:="${rviz_file}" &
launch_pid=$!

cleanup_runtime() {
  kill "${launch_pid}" >/dev/null 2>&1 || true
}
trap cleanup_runtime EXIT


# ==========================================
# 7. 捕获地图数据并保存为 2D 栅格
# ==========================================
echo "⏳ 等待点云被压平转换成 /map 话题 (这可能需要几秒钟)..."
# 轮询等待 /map 话题发布
timeout 10s bash -lc "until ros2 topic info /map >/dev/null 2>&1; do sleep 0.2; done" || true

echo "💾 正在生成 2D 栅格地图 (PGM/YAML)..."
ros2 run nav2_map_server map_saver_cli -t /map -f "${out_dir}/map" || true

echo "🎉 完美！2D 导航图已保存至: ${out_dir}"