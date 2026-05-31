#!/usr/bin/env python3
"""绘制轨迹精度对比图：EKF vs FAST-LIO2"""

import numpy as np
import matplotlib.pyplot as plt
import sys

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Noto Serif CJK SC', 'Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix'

ref_path = sys.argv[1] if len(sys.argv) > 1 else "benchmark_result/fastlio2_traj.txt"
est_path = sys.argv[2] if len(sys.argv) > 2 else "benchmark_result/ekf_traj.txt"
out_prefix = sys.argv[3] if len(sys.argv) > 3 else "benchmark_result/traj"

def quat_to_yaw(qx, qy, qz, qw):
    """四元数转 yaw 角 (rad)"""
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

def load_tum(path):
    data = np.loadtxt(path)
    t = data[:, 0]
    t = t - t[0]  # 归一化时间
    pos = data[:, 1:4]  # xyz
    quat = data[:, 4:8]  # qx qy qz qw
    yaw = quat_to_yaw(quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3])
    return t, pos, yaw

t_ref, pos_ref, yaw_ref = load_tum(ref_path)
t_est, pos_est, yaw_est = load_tum(est_path)

# 对齐：用线性插值把 est 对齐到 ref 的时间戳
from scipy.interpolate import interp1d
interp_x = interp1d(t_est, pos_est[:, 0], bounds_error=False, fill_value='extrapolate')
interp_y = interp1d(t_est, pos_est[:, 1], bounds_error=False, fill_value='extrapolate')
interp_yaw = interp1d(t_est, yaw_est, bounds_error=False, fill_value='extrapolate')
pos_est_aligned = np.column_stack([interp_x(t_ref), interp_y(t_ref)])
yaw_est_aligned = interp_yaw(t_ref)

# 计算逐帧位置误差
errors = np.sqrt((pos_ref[:, 0] - pos_est_aligned[:, 0])**2 +
                 (pos_ref[:, 1] - pos_est_aligned[:, 1])**2)

rmse = np.sqrt(np.mean(errors**2))
mean_err = np.mean(errors)
max_err = np.max(errors)
final_err = errors[-1]

# 计算 yaw 角误差（归一化到 [-pi, pi]）
yaw_errors = yaw_ref - yaw_est_aligned
yaw_errors = (yaw_errors + np.pi) % (2 * np.pi) - np.pi
yaw_errors_deg = np.degrees(yaw_errors)
yaw_rmse = np.sqrt(np.mean(yaw_errors_deg**2))
yaw_mean = np.mean(np.abs(yaw_errors_deg))
yaw_max = np.max(np.abs(yaw_errors_deg))

# === 图1: XY 轨迹对比 ===
fig1, ax1 = plt.subplots(figsize=(7, 5))
ax1.plot(pos_ref[:, 0], pos_ref[:, 1], label='激光惯性紧耦合里程计 (参考基准)', linewidth=2, color='#e74c3c')
ax1.plot(pos_est_aligned[:, 0], pos_est_aligned[:, 1], label='轻量化局部状态估计', linewidth=2, color='#3498db')
ax1.set_xlabel('X (m)', fontsize=13)
ax1.set_ylabel('Y (m)', fontsize=13)
ax1.set_title('行驶轨迹对比', fontsize=15)
ax1.legend(fontsize=12)
ax1.grid(True, alpha=0.3)
# 标注终点坐标
ax1.plot(pos_ref[-1, 0], pos_ref[-1, 1], 'o', color='#e74c3c', markersize=6)
ax1.annotate(f'({pos_ref[-1, 0]:.2f}, {pos_ref[-1, 1]:.2f})',
             xy=(pos_ref[-1, 0], pos_ref[-1, 1]), fontsize=10, color='#e74c3c',
             xytext=(-80, 10), textcoords='offset points')
ax1.plot(pos_est_aligned[-1, 0], pos_est_aligned[-1, 1], 'o', color='#3498db', markersize=6)
ax1.annotate(f'({pos_est_aligned[-1, 0]:.2f}, {pos_est_aligned[-1, 1]:.2f})',
             xy=(pos_est_aligned[-1, 0], pos_est_aligned[-1, 1]), fontsize=10, color='#3498db',
             xytext=(-40, 20), textcoords='offset points')
plt.tight_layout()
fig1.savefig(f"{out_prefix}_xy.svg")
print(f"Saved: {out_prefix}_xy.svg")

# === 图2: 位置误差随时间变化 ===
fig2, ax2 = plt.subplots(figsize=(7, 5))
ax2.plot(t_ref, errors, linewidth=1.5, color='#2ecc71', label='逐时刻误差')
ax2.axhline(rmse, color='#e74c3c', linestyle='--', alpha=0.7, label=f'均方根误差 = {rmse:.3f} m')
ax2.axhline(mean_err, color='#3498db', linestyle='--', alpha=0.7, label=f'平均误差 = {mean_err:.3f} m')
ax2.set_xlabel('时间 (s)', fontsize=13)
ax2.set_ylabel('位置误差 (m)', fontsize=13)
ax2.set_title('位置误差', fontsize=15)
ax2.legend(fontsize=12)
ax2.grid(True, alpha=0.3)
ax2.set_ylim(bottom=-0.01)
plt.tight_layout()
fig2.savefig(f"{out_prefix}_error.svg")
print(f"Saved: {out_prefix}_error.svg")

# === 图3: yaw 角误差随时间变化 ===
fig3, ax3 = plt.subplots(figsize=(7, 5))
ax3.plot(t_ref, yaw_errors_deg, linewidth=1.5, color='#2ecc71', label='逐时刻误差')
ax3.axhline(yaw_rmse, color='#e74c3c', linestyle='--', alpha=0.7, label=f'均方根误差 = {yaw_rmse:.3f}°')
ax3.axhline(-yaw_rmse, color='#e74c3c', linestyle='--', alpha=0.7)
ax3.axhline(0, color='gray', linestyle='-', alpha=0.3)
ax3.set_xlabel('时间 (s)', fontsize=13)
ax3.set_ylabel('航向角误差 (°)', fontsize=13)
ax3.set_title('航向角误差', fontsize=15)
ax3.legend(fontsize=12)
ax3.grid(True, alpha=0.3)
plt.tight_layout()
fig3.savefig(f"{out_prefix}_yaw.svg")
print(f"Saved: {out_prefix}_yaw.svg")

# === 汇总 ===
print(f"\n=== Trajectory Accuracy (0~94s, Forward Path) ===")
print(f"位置 RMSE:  {rmse:.4f} m")
print(f"位置 Mean:  {mean_err:.4f} m")
print(f"位置 Max:   {max_err:.4f} m")
print(f"位置 Final: {final_err:.4f} m (at x={pos_ref[-1, 0]:.2f} m)")
print(f"航向 RMSE:  {yaw_rmse:.3f}°")
print(f"航向 Mean:  {yaw_mean:.3f}°")
print(f"航向 Max:   {yaw_max:.3f}°")
