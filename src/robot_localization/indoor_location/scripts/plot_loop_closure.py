#!/usr/bin/env python3
"""回环闭合实验数据可视化脚本

用法:
    python3 plot_loop_closure.py <bag_dir> [--output loop_closure_report.svg]

输出 SVG 格式的实验报告图。
"""

import argparse
import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# 导入分析函数
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_loop_closure import extract_trajectory, compute_ate, compute_rpe, normalize_angle


def plot_report(poses, output_path):
    """绘制实验报告 SVG"""
    # 准备数据
    timestamps = np.array([p[0] for p in poses])
    x = np.array([p[1] for p in poses])
    y = np.array([p[2] for p in poses])
    yaw = np.array([p[3] for p in poses])

    # 相对时间（秒）
    t = timestamps - timestamps[0]

    # ATE
    ate = compute_ate(poses)
    ate = np.array(ate)

    # RPE
    rpe_results = compute_rpe(poses, delta=1)
    rpe_trans = np.array([e[0] for e in rpe_results])
    rpe_ang = np.array([e[1] for e in rpe_results])

    # 回环闭合误差
    loop_xy = np.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2)
    loop_yaw = abs(normalize_angle(yaw[-1] - yaw[0]))

    # 创建图形
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('Loop Closure Experiment Report', fontsize=16, fontweight='bold', y=0.98)

    gs = GridSpec(3, 2, figure=fig, hspace=0.35, wspace=0.3)

    # 1. 轨迹图 (左上)
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(x, y, 'b-', linewidth=0.8, label='Trajectory')
    ax1.plot(x[0], y[0], 'go', markersize=10, label='Start')
    ax1.plot(x[-1], y[-1], 'r*', markersize=15, label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Robot Trajectory (map frame)')
    ax1.legend(loc='best', fontsize=8)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # 2. ATE 随时间变化 (右上)
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, ate, 'b-', linewidth=0.8)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('ATE (m)')
    ax2.set_title('Absolute Trajectory Error')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=np.mean(ate), color='r', linestyle='--', alpha=0.7, label=f'Mean={np.mean(ate):.3f}m')
    ax2.legend(loc='best', fontsize=8)

    # 3. RPE 平移随时间变化 (左中)
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(t[1:], rpe_trans * 100, 'g-', linewidth=0.5, alpha=0.7)  # 转换为 cm
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('RPE Translation (cm)')
    ax3.set_title('Relative Pose Error - Translation')
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=np.mean(rpe_trans) * 100, color='r', linestyle='--', alpha=0.7,
                label=f'Mean={np.mean(rpe_trans)*100:.3f}cm')
    ax3.legend(loc='best', fontsize=8)

    # 4. RPE 旋转随时间变化 (右中)
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(t[1:], np.degrees(rpe_ang), 'm-', linewidth=0.5, alpha=0.7)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('RPE Rotation (deg)')
    ax4.set_title('Relative Pose Error - Rotation')
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=np.degrees(np.mean(rpe_ang)), color='r', linestyle='--', alpha=0.7,
                label=f'Mean={np.degrees(np.mean(rpe_ang)):.3f}deg')
    ax4.legend(loc='best', fontsize=8)

    # 5. Yaw 角随时间变化 (左下)
    ax5 = fig.add_subplot(gs[2, 0])
    ax5.plot(t, np.degrees(yaw), 'c-', linewidth=0.8)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Yaw (deg)')
    ax5.set_title('Heading Angle')
    ax5.grid(True, alpha=0.3)

    # 6. 统计摘要表 (右下)
    ax6 = fig.add_subplot(gs[2, 1])
    ax6.axis('off')

    # 计算统计量
    stats_data = [
        ['Metric', 'Value'],
        ['Duration', f'{t[-1]:.1f} s'],
        ['Poses', f'{len(poses)}'],
        ['', ''],
        ['Loop Closure XY', f'{loop_xy:.4f} m ({loop_xy*100:.1f} cm)'],
        ['Loop Closure Yaw', f'{np.degrees(loop_yaw):.2f} deg'],
        ['', ''],
        ['ATE RMSE', f'{np.sqrt(np.mean(ate**2)):.4f} m'],
        ['ATE Mean', f'{np.mean(ate):.4f} m'],
        ['ATE Median', f'{np.median(ate):.4f} m'],
        ['ATE Max', f'{np.max(ate):.4f} m'],
        ['', ''],
        ['RPE Trans RMSE', f'{np.sqrt(np.mean(rpe_trans**2))*100:.3f} cm'],
        ['RPE Trans Mean', f'{np.mean(rpe_trans)*100:.3f} cm'],
        ['RPE Trans Max', f'{np.max(rpe_trans)*100:.3f} cm'],
        ['', ''],
        ['RPE Rot RMSE', f'{np.degrees(np.sqrt(np.mean(rpe_ang**2))):.3f} deg'],
        ['RPE Rot Mean', f'{np.degrees(np.mean(rpe_ang)):.3f} deg'],
        ['RPE Rot Max', f'{np.degrees(np.max(rpe_ang)):.3f} deg'],
    ]

    # 绘制表格
    table = ax6.table(cellText=stats_data, loc='center', cellLoc='left',
                      colWidths=[0.45, 0.55])
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.3)

    # 设置表头样式
    for i in range(2):
        table[0, i].set_facecolor('#4CAF50')
        table[0, i].set_text_props(color='white', fontweight='bold')

    # 设置空行样式
    for i in range(len(stats_data)):
        if stats_data[i][0] == '' and stats_data[i][1] == '':
            for j in range(2):
                table[i, j].set_facecolor('#f0f0f0')
                table[i, j].set_edgecolor('#f0f0f0')

    ax6.set_title('Statistics Summary', fontsize=10, fontweight='bold', pad=10)

    # 保存 SVG
    plt.savefig(output_path, format='svg', dpi=150, bbox_inches='tight')
    print(f"Report saved to: {output_path}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(description="回环闭合实验数据可视化")
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

    # 输出路径
    if args.output is None:
        args.output = bag_path.rstrip('/') + '_report.svg'

    print(f"Analyzing bag: {bag_path}")

    # 提取轨迹
    poses = extract_trajectory(bag_path, args.map_frame)

    if len(poses) < 2:
        print("Error: insufficient poses extracted")
        sys.exit(1)

    print(f"Extracted {len(poses)} poses, duration {poses[-1][0]-poses[0][0]:.1f}s")

    # 绘制报告
    plot_report(poses, args.output)


if __name__ == "__main__":
    main()
