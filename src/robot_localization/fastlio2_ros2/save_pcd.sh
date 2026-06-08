#!/usr/bin/env bash

# ==========================================
# 1. 动态获取路径
# ==========================================
# 获取当前脚本所在的绝对路径 (.../src/robot_localization/fastlio2_ros2)
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 动态计算工作空间根目录 (向上退 3 级目录)
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
    echo "⚠️ 警告: 未找到 ${ws_dir}/install/setup.bash，请确保你已经 colcon build 编译过了！"
fi


# ==========================================
# 3. 开启严格模式
# ==========================================
# 放在 source 后面，防止 ROS 2 自带脚本触发未绑定变量的报错退出
set -euo pipefail


# ==========================================
# 4. 核心逻辑：调用服务保存地图
# ==========================================
# 设定保存地图的基础目录
base_dir="${script_dir}/map/pcd"

# 按时间戳生成文件夹名称
timestamp="$(date +%Y%m%d_%H%M%S)"
out_dir="${base_dir}/${timestamp}"

# 创建文件夹
mkdir -p "${out_dir}"

echo "Requesting Map Save to: ${out_dir} ..."

# 调用 ROS 2 服务并捕获输出
RESPONSE=$(ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '${out_dir}', save_patches: true}")

# 打印出服务返回的信息
echo "$RESPONSE"

# 判断返回值中是否包含 success=True
if [[ "$RESPONSE" == *"success=True"* ]]; then
    echo "✅ Map saved successfully to: ${out_dir}"
else
    echo "❌ Failed to save map! Reason: Map backend returned an error (e.g., NO POSES)."
    # 如果保存失败，把刚才建的空文件夹删掉，保持目录整洁
    rm -rf "${out_dir}"
fi