#!/usr/bin/env python3
"""全局定位 vs 轮式里程计对比可视化脚本

用法:
    python3 plot_comparison.py <bag_dir> [--output comparison_report.svg]

在同一张图上对比全局定位和轮式里程计的轨迹和误差。
"""

import argparse
import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_loop_closure import extract_trajectory, compute_ate, compute_rpe, normalize_angle

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def extract_odom_trajectory(bag_path):
    """从 bag 中提取轮式里程计轨迹（不加 map 修正）

    返回: [(stamp, x, y, yaw), ...]
    """
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    odom_list = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == '/odom':
            msg = deserialize_message(data, Odometry)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            if (np.isfinite(p.x) and np.isfinite(p.y) and
                abs(q.x) <= 1.0 and abs(q.y) <= 1.0 and abs(q.z) <= 1.0 and abs(q.w) <= 1.0):
                yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
                odom_list.append((stamp, p.x, p.y, yaw))

    print(f"Odom trajectory: {len(odom_list)} poses")
    return odom_list


def plot_comparison(global_poses, odom_poses, output_path):
    """绘制全局定位 vs 轮式里程计对比图"""
    # 对齐时间戳（以全局定位为基准）
    g_t = np.array([p[0] for p in global_poses])
    g_x = np.array([p[1] for p in global_poses])
    g_y = np.array([p[2] for p in global_poses])
    g_yaw = np.array([p[3] for p in global_poses])
    g_t = g_t - g_t[0]

    o_t = np.array([p[0] for p in odom_poses])
    o_x = np.array([p[1] for p in odom_poses])
    o_y = np.array([p[2] for p in odom_poses])
    o_yaw = np.array([p[3] for p in odom_poses])
    o_t = o_t - o_t[0]

    # ATE
    g_ate = np.array(compute_ate(global_poses))
    o_ate = np.array(compute_ate(odom_poses))

    # RPE
    g_rpe = compute_rpe(global_poses)
    o_rpe = compute_rpe(odom_poses)
    g_rpe_t = np.array([e[0] for e in g_rpe])
    o_rpe_t = np.array([e[0] for e in o_rpe])

    # 创建图形
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('Global Localization vs Wheel Odometry Comparison', fontsize=14, fontweight='bold', y=0.98)

    gs = GridSpec(2, 2, figure=fig, hspace=0.35, wspace=0.3)

    # 1. 轨迹对比图 (左上)
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(o_x, o_y, 'r-', linewidth=0.8, alpha=0.7, label='Wheel Odometry')
    ax1.plot(g_x, g_y, 'b-', linewidth=0.8, alpha=0.7, label='Global Localization')
    ax1.plot(g_x[0], g_y[0], 'go', markersize=10, label='Start')
    ax1.plot(g_x[-1], g_y[-1], 'b*', markersize=12)
    ax1.plot(o_x[-1], o_y[-1], 'r*', markersize=12)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Trajectory Comparison')
    ax1.legend(loc='best', fontsize=8)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # 2. ATE 对比 (右上)
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(o_t, o_ate, 'r-', linewidth=0.8, alpha=0.7, label='Wheel Odometry')
    ax2.plot(g_t, g_ate, 'b-', linewidth=0.8, alpha=0.7, label='Global Localization')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('ATE (m)')
    ax2.set_title('Absolute Trajectory Error Comparison')
    ax2.legend(loc='best', fontsize=8)
    ax2.grid(True, alpha=0.3)

    # 3. RPE 平移对比 (左下)
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(o_t[1:], o_rpe_t * 100, 'r-', linewidth=0.5, alpha=0.5, label='Wheel Odometry')
    ax3.plot(g_t[1:], g_rpe_t * 100, 'b-', linewidth=0.5, alpha=0.5, label='Global Localization')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('RPE Translation (cm)')
    ax3.set_title('Relative Pose Error - Translation')
    ax3.legend(loc='best', fontsize=8)
    ax3.grid(True, alpha=0.3)

    # 4. 统计对比表 (右下)
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.axis('off')

    # 计算统计量
    def get_stats(arr):
        return {
            'rmse': np.sqrt(np.mean(arr**2)),
            'mean': np.mean(arr),
            'median': np.median(arr),
            'max': np.max(arr)
        }

    g_ate_stats = get_stats(g_ate)
    o_ate_stats = get_stats(o_ate)
    g_rpe_stats = get_stats(g_rpe_t * 100)  # cm
    o_rpe_stats = get_stats(o_rpe_t * 100)  # cm

    # 回环闭合误差
    g_loop = np.sqrt((g_x[-1] - g_x[0])**2 + (g_y[-1] - g_y[0])**2)
    o_loop = np.sqrt((o_x[-1] - o_x[0])**2 + (o_y[-1] - o_y[0])**2)

    stats_data = [
        ['Metric', 'Global Loc', 'Wheel Odom', 'Improvement'],
        ['Loop Closure (m)', f'{g_loop:.3f}', f'{o_loop:.3f}', f'{(1-g_loop/o_loop)*100:.1f}%' if o_loop > 0 else 'N/A'],
        ['', '', '', ''],
        ['ATE RMSE (m)', f'{g_ate_stats["rmse"]:.3f}', f'{o_ate_stats["rmse"]:.3f}',
         f'{(1-g_ate_stats["rmse"]/o_ate_stats["rmse"])*100:.1f}%' if o_ate_stats["rmse"] > 0 else 'N/A'],
        ['ATE Mean (m)', f'{g_ate_stats["mean"]:.3f}', f'{o_ate_stats["mean"]:.3f}',
         f'{(1-g_ate_stats["mean"]/o_ate_stats["mean"])*100:.1f}%' if o_ate_stats["mean"] > 0 else 'N/A'],
        ['ATE Max (m)', f'{g_ate_stats["max"]:.3f}', f'{o_ate_stats["max"]:.3f}',
         f'{(1-g_ate_stats["max"]/o_ate_stats["max"])*100:.1f}%' if o_ate_stats["max"] > 0 else 'N/A'],
        ['', '', '', ''],
        ['RPE Trans RMSE (cm)', f'{g_rpe_stats["rmse"]:.3f}', f'{o_rpe_stats["rmse"]:.3f}',
         f'{(1-g_rpe_stats["rmse"]/o_rpe_stats["rmse"])*100:.1f}%' if o_rpe_stats["rmse"] > 0 else 'N/A'],
        ['RPE Trans Mean (cm)', f'{g_rpe_stats["mean"]:.3f}', f'{o_rpe_stats["mean"]:.3f}',
         f'{(1-g_rpe_stats["mean"]/o_rpe_stats["mean"])*100:.1f}%' if o_rpe_stats["mean"] > 0 else 'N/A'],
    ]

    table = ax4.table(cellText=stats_data, loc='center', cellLoc='center',
                      colWidths=[0.3, 0.22, 0.22, 0.22])
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.4)

    # 设置表头样式
    for i in range(4):
        table[0, i].set_facecolor('#2196F3')
        table[0, i].set_text_props(color='white', fontweight='bold')

    # 设置空行样式
    for i in range(len(stats_data)):
        if stats_data[i][0] == '':
            for j in range(4):
                table[i, j].set_facecolor('#f0f0f0')
                table[i, j].set_edgecolor('#f0f0f0')

    ax4.set_title('Statistics Comparison', fontsize=10, fontweight='bold', pad=10)

    # 保存 SVG
    plt.savefig(output_path, format='svg', dpi=150, bbox_inches='tight')
    print(f"Comparison report saved to: {output_path}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(description="全局定位 vs 轮式里程计对比")
    parser.add_argument("bag_dir", help="bag 目录或 .db3 文件路径")
    parser.add_argument("--output", "-o", default=None, help="输出 SVG 文件路径")
    parser.add_argument("--map-frame", default="map", help="地图坐标系 (default: map)")
    args = parser.parse_args()

    # 找 bag 目录
    bag_path = args.bag_dir
    if os.path.isfile(bag_path) and bag_path.endswith('.db3'):
        bag_path = os.path.dirname(bag_path)

    if not os.path.exists(bag_path):
        print(f"Error: bag path '{bag_path}' does not exist")
        sys.exit(1)

    if args.output is None:
        args.output = bag_path.rstrip('/') + '_comparison.svg'

    print(f"Analyzing bag: {bag_path}")

    # 提取全局定位轨迹
    global_poses = extract_trajectory(bag_path, args.map_frame)
    if len(global_poses) < 2:
        print("Error: insufficient global poses")
        sys.exit(1)

    # 提取轮式里程计轨迹
    odom_poses = extract_odom_trajectory(bag_path)
    if len(odom_poses) < 2:
        print("Error: insufficient odom poses")
        sys.exit(1)

    print(f"Global: {len(global_poses)} poses, Odom: {len(odom_poses)} poses")

    # 绘制对比图
    plot_comparison(global_poses, odom_poses, args.output)


if __name__ == "__main__":
    main()
