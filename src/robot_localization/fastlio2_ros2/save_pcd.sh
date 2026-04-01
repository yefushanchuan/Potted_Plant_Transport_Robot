#!/usr/bin/env bash
set -euo pipefail

# 自动获取当前脚本所在的绝对路径
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 指向你的工程目录下的 map/pcd 文件夹
base_dir="${script_dir}/map/pcd"

# 按时间戳生成文件夹名称
timestamp="$(date +%Y%m%d_%H%M%S)"
out_dir="${base_dir}/${timestamp}"

# 创建文件夹
mkdir -p "${out_dir}"

echo "Requesting Map Save to: ${out_dir} ..."

# 调用 ROS 2 服务并捕获输出
RESPONSE=$(ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '${out_dir}', save_patches: false}")

# 打印出服务返回的信息
echo "$RESPONSE"

# 判断返回值中是否包含 success=True
if [[ "$RESPONSE" == *"success=True"* ]]; then
    echo "✅ Map saved successfully to: ${out_dir}"
else
    echo "❌ Failed to save map! Reason: Map backend returned an error (e.g., NO POSES)."
    # 如果保存失败，可以选择把刚才建的空文件夹删掉
    rm -rf "${out_dir}"
fi