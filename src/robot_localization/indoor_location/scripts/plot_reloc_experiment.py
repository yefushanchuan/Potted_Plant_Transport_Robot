#!/usr/bin/env python3
"""重定位实验可视化脚本

用法:
    python3 plot_reloc_experiment.py <csv1> [csv2 ...] [--output reloc_report.svg]

从 CSV 数据中提取恢复事件，分类统计并绘制 2 张子图：
  1. 分组柱状图 — 平均误差 & 平均耗时
  2. 散点图 — 误差 vs 耗时
"""

import argparse
import csv
import math
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Noto Serif CJK SC', 'Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix'


def parse_recoveries(fpath):
    """从 CSV 中提取恢复事件。"""
    rows = []
    with open(fpath) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not row['timestamp'].strip():
                continue
            try:
                rows.append({
                    'ts': float(row['timestamp']),
                    'status': row['status'].strip(),
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'yaw': float(row['yaw']),
                    'has_icp': int(row['has_icp_pose']),
                })
            except (ValueError, KeyError):
                continue

    recoveries = []
    i = 0
    while i < len(rows):
        if rows[i]['status'] == '1':
            pre_lost_idx = i - 1
            while pre_lost_idx >= 0 and rows[pre_lost_idx]['status'] == '1':
                pre_lost_idx -= 1
            j = i
            while j < len(rows) and rows[j]['status'] in ('1', 'FORCE_LOST'):
                j += 1
            if j < len(rows) and rows[j]['status'] == '0':
                duration = rows[j]['ts'] - rows[i]['ts']
                if pre_lost_idx >= 0:
                    pre_x, pre_y = rows[pre_lost_idx]['x'], rows[pre_lost_idx]['y']
                else:
                    pre_x, pre_y = rows[0]['x'], rows[0]['y']
                post_x, post_y = rows[j]['x'], rows[j]['y']
                error = math.sqrt((post_x - pre_x)**2 + (post_y - pre_y)**2)
                had_icp = any(rows[k]['has_icp'] == 1 for k in range(i, j + 1))
                recoveries.append({
                    'duration': duration,
                    'error': error,
                    'had_icp': had_icp,
                    'pre_pos': (pre_x, pre_y),
                    'post_pos': (post_x, post_y),
                    'file': fpath.split('/')[-1],
                })
                i = j + 1
            else:
                i = j
        else:
            i += 1
    return recoveries


def classify(recoveries, threshold=0.3):
    """按耗时阈值分类：<=threshold 为自恢复，>threshold 为副节点辅助。"""
    self_r = [r for r in recoveries if r['duration'] <= threshold]
    assist_r = [r for r in recoveries if r['duration'] > threshold]
    return self_r, assist_r


def plot_bar_chart(ax, self_r, assist_r):
    """分组柱状图：平均误差 & 平均耗时。"""
    metrics = ['平均定位误差 (m)', '平均恢复耗时 (s)']
    self_vals = [np.mean([r['error'] for r in self_r]),
                 np.mean([r['duration'] for r in self_r])]
    assist_vals = [np.mean([r['error'] for r in assist_r]),
                   np.mean([r['duration'] for r in assist_r])]

    x = np.arange(len(metrics))
    w = 0.3
    bars1 = ax.bar(x - w/2, self_vals, w, label=f'主节点自恢复 ({len(self_r)}次)',
                   color='#4CAF50', edgecolor='white')
    bars2 = ax.bar(x + w/2, assist_vals, w, label=f'副节点辅助 ({len(assist_r)}次)',
                   color='#FF9800', edgecolor='white')

    for bar in bars1:
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.005,
                f'{bar.get_height():.3f}', ha='center', va='bottom', fontsize=10)
    for bar in bars2:
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.005,
                f'{bar.get_height():.3f}', ha='center', va='bottom', fontsize=10)

    ax.set_xticks(x)
    ax.set_xticklabels(metrics, fontsize=12)
    ax.set_ylabel('数值', fontsize=13)
    ax.set_title('恢复性能对比', fontsize=15)
    ax.legend(fontsize=11)
    ax.grid(axis='y', alpha=0.3)
    ax.set_ylim(bottom=0)


def plot_scatter(ax, self_r, assist_r):
    """散点图：误差 vs 耗时。"""
    ax.scatter([r['duration'] for r in self_r],
               [r['error'] for r in self_r],
               c='#4CAF50', alpha=0.6, s=40, label=f'主节点自恢复 ({len(self_r)}次)', zorder=3)
    ax.scatter([r['duration'] for r in assist_r],
               [r['error'] for r in assist_r],
               c='#FF9800', alpha=0.6, s=40, label=f'副节点辅助 ({len(assist_r)}次)',
               marker='s', zorder=3)

    ax.axvline(x=0.3, color='gray', linestyle='--', alpha=0.5, label='分类阈值 (0.3s)')
    ax.set_xlabel('恢复耗时 (s)', fontsize=13)
    ax.set_ylabel('定位误差 (m)', fontsize=13)
    ax.set_title('误差与恢复耗时分布', fontsize=15)
    ax.legend(fontsize=11)
    ax.grid(alpha=0.3)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)


def main():
    parser = argparse.ArgumentParser(description="重定位实验可视化")
    parser.add_argument("csv_files", nargs='+', help="CSV 数据文件")
    parser.add_argument("--output", "-o", default="reloc_experiment_report.svg", help="输出 SVG 文件路径")
    parser.add_argument("--exclude", nargs='*', default=None,
                        help="排除的恢复序号，格式: filename:start-end [...]")
    args = parser.parse_args()

    # 解析排除范围
    exclude_ranges = {}
    if args.exclude:
        for item in args.exclude:
            fname, rng = item.split(':')
            start, end = rng.split('-')
            exclude_ranges.setdefault(fname, []).append((int(start), int(end)))

    # 加载数据
    print("Loading data...")
    all_recoveries = []
    for fpath in args.csv_files:
        recs = parse_recoveries(fpath)
        fname = fpath.split('/')[-1]
        for idx, r in enumerate(recs):
            r['idx'] = idx + 1
        if fname in exclude_ranges:
            orig_len = len(recs)
            for start, end in exclude_ranges[fname]:
                recs = [r for r in recs if not (start <= r['idx'] <= end)]
            print(f"  {fname}: {orig_len} -> {len(recs)} 次恢复 (排除 {orig_len - len(recs)} 次)")
        else:
            print(f"  {fname}: {len(recs)} 次恢复")
        all_recoveries.extend(recs)

    self_r, assist_r = classify(all_recoveries)
    total = len(all_recoveries)

    print(f"\n总计: {total} 次 | 自恢复: {len(self_r)} ({100*len(self_r)/total:.0f}%) | "
          f"副节点辅助: {len(assist_r)} ({100*len(assist_r)/total:.0f}%)")
    print(f"自恢复平均误差: {np.mean([r['error'] for r in self_r]):.3f}m | "
          f"副节点平均误差: {np.mean([r['error'] for r in assist_r]):.3f}m")

    # 绘图
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5))

    plot_bar_chart(ax1, self_r, assist_r)
    plot_scatter(ax2, self_r, assist_r)

    plt.tight_layout()
    plt.savefig(args.output, dpi=150, bbox_inches='tight')
    print(f"\n报告已保存: {args.output}")
    plt.close()


if __name__ == "__main__":
    main()
