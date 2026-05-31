#!/usr/bin/env python3
"""绘制发布延迟对比图：FAST-LIO2 vs EKF"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Noto Serif CJK SC', 'Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix'

csv_path = sys.argv[1] if len(sys.argv) > 1 else "benchmark_result/delay.csv"
out_path = sys.argv[2] if len(sys.argv) > 2 else "benchmark_result/delay.svg"

df = pd.read_csv(csv_path)
df['timestamp'] = pd.to_datetime(df['timestamp'])

fastlio2 = df[df['node'] == 'fastlio2'].copy()
ekf = df[df['node'] == 'ekf'].copy()

t0 = min(fastlio2['timestamp'].iloc[0], ekf['timestamp'].iloc[0])
fastlio2['t'] = (fastlio2['timestamp'] - t0).dt.total_seconds()
ekf['t'] = (ekf['timestamp'] - t0).dt.total_seconds()
fastlio2 = fastlio2[fastlio2['t'] <= 200]
ekf = ekf[ekf['t'] <= 200]

fig, ax = plt.subplots(figsize=(7, 5))
t_f = fastlio2['t'].values
t_e = ekf['t'].values
ax.plot(t_f, fastlio2['avg_delay_ms'].values, label='激光惯性紧耦合里程计', linewidth=1.5, color='#e74c3c')
ax.plot(t_e, ekf['avg_delay_ms'].values, label='轻量化局部状态估计', linewidth=1.5, color='#3498db')
ax.set_xlabel('时间 (s)', fontsize=13)
ax.set_ylabel('发布延迟 (ms)', fontsize=13)
ax.set_title('发布延迟对比', fontsize=15)
ax.legend(fontsize=12)
ax.grid(True, alpha=0.3)

ax.set_xticks(np.arange(0, 201, 50))

avg_f = fastlio2['avg_delay_ms'].mean()
avg_e = ekf['avg_delay_ms'].mean()
ax.axhline(avg_f, color='#e74c3c', linestyle='--', alpha=0.5)
ax.axhline(avg_e, color='#3498db', linestyle='--', alpha=0.5)
ax.text(t_f[-1]*0.69, avg_f+0.001, f'平均发布延迟 {avg_f:.3f} ms', fontsize=10, color='#e74c3c')
ax.text(t_e[-1]*0.7, avg_e+0.001, f'平均发布延迟 {avg_e:.3f} ms', fontsize=10, color='#3498db')

plt.tight_layout()
plt.savefig(out_path, dpi=150)
print(f"Saved: {out_path}")
print(f"FAST-LIO2 avg: {avg_f:.3f} ms, EKF avg: {avg_e:.3f} ms")
