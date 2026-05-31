#!/usr/bin/env python3
"""绘制 CPU 占用率对比图：FAST-LIO2 vs EKF"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Noto Serif CJK SC', 'Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix'

csv_path = sys.argv[1] if len(sys.argv) > 1 else "benchmark_result/cpu.csv"
out_path = sys.argv[2] if len(sys.argv) > 2 else "benchmark_result/cpu.svg"

df = pd.read_csv(csv_path)
df['timestamp'] = pd.to_datetime(df['timestamp'])
# 只保留两个节点都在运行的数据
df = df[(df['fastlio2_cpu%'] > 0) | (df['ekf_cpu%'] > 0)]
df['t'] = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()
df = df[df['t'] <= 200]

fig, ax = plt.subplots(figsize=(7, 5))
t = df['t'].values
ax.plot(t, df['fastlio2_cpu%'].values, label='激光惯性紧耦合里程计', linewidth=2, color='#e74c3c')
ax.plot(t, df['ekf_cpu%'].values, label='轻量化局部状态估计', linewidth=2, color='#3498db')
ax.set_xlabel('时间 (s)', fontsize=13)
ax.set_ylabel('CPU 占用 (%)', fontsize=13)
ax.set_title('CPU 占用对比', fontsize=15)
ax.legend(fontsize=12)
ax.grid(True, alpha=0.3)
ax.set_xticks(np.arange(0, 201, 50))
ax.set_ylim(bottom=0)

avg_fastlio2 = df['fastlio2_cpu%'].mean()
avg_ekf = df['ekf_cpu%'].mean()
ax.axhline(avg_fastlio2, color='#e74c3c', linestyle='--', alpha=0.5)
ax.axhline(avg_ekf, color='#3498db', linestyle='--', alpha=0.5)
ax.text(t[-1]*0.7, avg_fastlio2+1, f'平均占用率 {avg_fastlio2:.1f}%', fontsize=10, color='#e74c3c')
ax.text(t[-1]*0.7, avg_ekf+1, f'平均占用率 {avg_ekf:.1f}%', fontsize=10, color='#3498db')

plt.tight_layout()
plt.savefig(out_path, dpi=150)
print(f"Saved: {out_path}")
print(f"FAST-LIO2 : {avg_fastlio2:.1f}%, EKF avg: {avg_ekf:.1f}%, ratio: {avg_fastlio2/avg_ekf:.0f}x")
