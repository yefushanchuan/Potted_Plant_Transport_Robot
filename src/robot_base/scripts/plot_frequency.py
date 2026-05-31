#!/usr/bin/env python3
"""绘制输出频率对比图：FAST-LIO2 vs EKF"""

import pandas as pd
import matplotlib.pyplot as plt
import sys

plt.rcParams['font.family'] = 'sans-serif'

csv_path = sys.argv[1] if len(sys.argv) > 1 else "benchmark_result/frequency.csv"
out_path = sys.argv[2] if len(sys.argv) > 2 else "benchmark_result/frequency.svg"

df = pd.read_csv(csv_path)
df['timestamp'] = pd.to_datetime(df['timestamp'])

fastlio2 = df[df['node'] == 'fastlio2'].copy()
ekf = df[df['node'] == 'ekf'].copy()

t0 = min(fastlio2['timestamp'].iloc[0], ekf['timestamp'].iloc[0])
fastlio2['t'] = (fastlio2['timestamp'] - t0).dt.total_seconds()
ekf['t'] = (ekf['timestamp'] - t0).dt.total_seconds()
fastlio2 = fastlio2[fastlio2['t'] <= 200]
ekf = ekf[ekf['t'] <= 200]

fig, ax = plt.subplots(figsize=(10, 5))
t_f = fastlio2['t'].values
t_e = ekf['t'].values
ax.plot(t_f, fastlio2['avg_rate_hz'].values, label='FAST-LIO2', linewidth=1.5, color='#e74c3c')
ax.plot(t_e, ekf['avg_rate_hz'].values, label='EKF (Wheel Odom + IMU)', linewidth=1.5, color='#3498db')
ax.set_xlabel('Time (s)', fontsize=13)
ax.set_ylabel('Output Frequency (Hz)', fontsize=13)
ax.set_title('Output Frequency: FAST-LIO2 vs EKF', fontsize=15)
ax.legend(fontsize=12)
ax.grid(True, alpha=0.3)

avg_f = fastlio2['avg_rate_hz'].mean()
avg_e = ekf['avg_rate_hz'].mean()
ax.axhline(avg_f, color='#e74c3c', linestyle='--', alpha=0.5)
ax.axhline(avg_e, color='#3498db', linestyle='--', alpha=0.5)
ax.text(t_f[-1]*0.7, avg_f+0.3, f'avg {avg_f:.1f} Hz', fontsize=10, color='#e74c3c')
ax.text(t_e[-1]*0.7, avg_e+0.3, f'avg {avg_e:.1f} Hz', fontsize=10, color='#3498db')

plt.tight_layout()
plt.savefig(out_path, dpi=150)
print(f"Saved: {out_path}")
print(f"FAST-LIO2 avg: {avg_f:.1f} Hz, EKF avg: {avg_e:.1f} Hz")
