#!/usr/bin/env python3
"""
导航精度评估脚本

用法:
  python3 eval_nav_accuracy.py --gt_bag nav_ground_truth --nav_bag nav_online --output result.png

从两个 rosbag 中提取轨迹:
  - 真值: TF map -> base_footprint (来自 FAST-LIO2 + BA)
  - 待评估: /agv_pose (来自定位节点)

输出: ATE/RPE 指标 + 轨迹对比图
"""

import argparse
import numpy as np
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

import tf_transformations as tft
from scipy.spatial.transform import Rotation


def read_pose_topic(bag_path, topic_name):
    """从 bag 中读取 PoseStamped topic，返回 [(timestamp_sec, x, y, yaw), ...]"""
    store = get_typestore(Stores.ROS2_HUMBLE)
    poses = []
    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic_name:
                msg = store.deserialize_cdr(rawdata, connection.msgtype)
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                x = msg.pose.position.x
                y = msg.pose.position.y
                q = msg.pose.orientation
                _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
                poses.append((t, x, y, yaw))
    return poses


def read_tf_trajectory(bag_path, parent_frame='map', child_frame='base_footprint'):
    """从 bag 中读取 TF，提取 map -> base_footprint 轨迹，返回 [(timestamp_sec, x, y, yaw), ...]"""
    store = get_typestore(Stores.ROS2_HUMBLE)
    poses = []
    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/tf':
                msg = store.deserialize_cdr(rawdata, connection.msgtype)
                for transform in msg.transforms:
                    if transform.header.frame_id == parent_frame and \
                       transform.child_frame_id == child_frame:
                        t = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                        x = transform.transform.translation.x
                        y = transform.transform.translation.y
                        q = transform.transform.rotation
                        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
                        poses.append((t, x, y, yaw))
    poses.sort(key=lambda p: p[0])
    return poses


def interpolate_trajectory(poses, timestamps):
    """将轨迹插值到指定时间戳列表，返回 [(t, x, y, yaw), ...]"""
    if len(poses) < 2:
        return []

    src_t = np.array([p[0] for p in poses])
    src_x = np.array([p[1] for p in poses])
    src_y = np.array([p[2] for p in poses])
    src_yaw = np.array([p[3] for p in poses])

    result = []
    for t in timestamps:
        if t < src_t[0] or t > src_t[-1]:
            continue
        # 找到左右两个时间点
        idx = np.searchsorted(src_t, t) - 1
        idx = max(0, min(idx, len(src_t) - 2))
        alpha = (t - src_t[idx]) / (src_t[idx + 1] - src_t[idx] + 1e-12)

        x = src_x[idx] + alpha * (src_x[idx + 1] - src_x[idx])
        y = src_y[idx] + alpha * (src_y[idx + 1] - src_y[idx])

        # yaw 角度插值 (处理 ±π 跳变)
        dyaw = src_yaw[idx + 1] - src_yaw[idx]
        dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
        yaw = src_yaw[idx] + alpha * dyaw

        result.append((t, x, y, yaw))
    return result


def align_trajectories(gt, est):
    """时间对齐两条轨迹，返回配对后的 (gt_aligned, est_aligned)"""
    gt_times = set(p[0] for p in gt)
    est_times = set(p[0] for p in est)

    # 以真值时间戳为基准，对估计轨迹做插值
    common_times = sorted(gt_times.intersection(est_times))

    # 如果完全重叠时间戳太少，用插值
    if len(common_times) < 10:
        gt_t = [p[0] for p in gt]
        est_t = [p[0] for p in est]
        overlap_start = max(gt_t[0], est_t[0])
        overlap_end = min(gt_t[-1], est_t[-1])
        if overlap_start >= overlap_end:
            print("错误: 两条轨迹没有时间重叠!")
            return [], []

        # 以 0.05s 间隔采样
        sample_times = np.arange(overlap_start, overlap_end, 0.05)
        gt_interp = interpolate_trajectory(gt, sample_times)
        est_interp = interpolate_trajectory(est, sample_times)
        return gt_interp, est_interp

    gt_dict = {p[0]: p for p in gt}
    est_dict = {p[0]: p for p in est}
    gt_aligned = [gt_dict[t] for t in common_times]
    est_aligned = [est_dict[t] for t in common_times]
    return gt_aligned, est_aligned


def compute_ate(gt, est):
    """计算绝对轨迹误差 (ATE): 仅平移部分"""
    gt_arr = np.array([[p[1], p[2]] for p in gt])
    est_arr = np.array([[p[1], p[2]] for p in est])

    # Umeyama 对齐 (相似变换: R, t, s)
    errors = np.linalg.norm(gt_arr - est_arr, axis=1)
    return {
        'rmse': np.sqrt(np.mean(errors ** 2)),
        'mean': np.mean(errors),
        'median': np.median(errors),
        'max': np.max(errors),
        'std': np.std(errors),
        'errors': errors,
    }


def compute_rpe(gt, est, delta=1.0):
    """计算相对位姿误差 (RPE): 间隔 delta 秒的相对运动误差"""
    if len(gt) < 2:
        return {'rmse': 0, 'mean': 0, 'median': 0}

    trans_errors = []
    rot_errors = []

    for i in range(len(gt) - 1):
        dt = gt[i + 1][0] - gt[i][0]
        if dt < 0.5 or dt > 2.0:
            continue

        # 真值相对运动
        dx_gt = gt[i + 1][1] - gt[i][1]
        dy_gt = gt[i + 1][2] - gt[i][2]
        dyaw_gt = gt[i + 1][3] - gt[i][3]
        dyaw_gt = (dyaw_gt + np.pi) % (2 * np.pi) - np.pi

        # 估计相对运动
        dx_est = est[i + 1][1] - est[i][1]
        dy_est = est[i + 1][2] - est[i][2]
        dyaw_est = est[i + 1][3] - est[i][3]
        dyaw_est = (dyaw_est + np.pi) % (2 * np.pi) - np.pi

        # 将估计的相对运动旋转到真值坐标系下比较
        cos_gt = np.cos(gt[i][3])
        sin_gt = np.sin(gt[i][3])
        cos_est = np.cos(est[i][3])
        sin_est = np.sin(est[i][3])

        # 相对运动误差
        dtrans = np.sqrt((dx_gt - dx_est) ** 2 + (dy_gt - dy_est) ** 2)
        drot = abs(dyaw_gt - dyaw_est)

        trans_errors.append(dtrans)
        rot_errors.append(drot)

    if not trans_errors:
        return {'rmse': 0, 'mean': 0, 'median': 0, 'rot_rmse_deg': 0}

    return {
        'rmse': np.sqrt(np.mean(np.array(trans_errors) ** 2)),
        'mean': np.mean(trans_errors),
        'median': np.median(trans_errors),
        'rot_rmse_deg': np.degrees(np.sqrt(np.mean(np.array(rot_errors) ** 2))),
    }


def plot_trajectories(gt, est, ate_errors, output_path):
    """绘制轨迹对比图"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    gt_x = [p[1] for p in gt]
    gt_y = [p[2] for p in gt]
    est_x = [p[1] for p in est]
    est_y = [p[2] for p in est]

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # 左图: 轨迹对比
    ax1 = axes[0]
    ax1.plot(gt_x, gt_y, 'b-', linewidth=1.5, label='Ground Truth (FAST-LIO2+BA)', alpha=0.8)
    ax1.plot(est_x, est_y, 'r-', linewidth=1.0, label='Navigation Pose (/agv_pose)', alpha=0.8)
    ax1.plot(gt_x[0], gt_y[0], 'go', markersize=10, label='Start')
    ax1.plot(gt_x[-1], gt_y[-1], 'rs', markersize=10, label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Trajectory Comparison')
    ax1.legend()
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # 右图: ATE 误差分布
    ax2 = axes[1]
    gt_arr = np.array([[p[1], p[2]] for p in gt])
    sc = ax2.scatter(gt_arr[:len(ate_errors), 0], gt_arr[:len(ate_errors), 1],
                     c=ate_errors, cmap='RdYlGn_r', s=8, vmin=0)
    plt.colorbar(sc, ax=ax2, label='ATE (m)')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('ATE Error Distribution')
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"\n轨迹图已保存: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='导航精度评估')
    parser.add_argument('--gt_bag', required=True, help='真值 TF bag 路径 (FAST-LIO2+BA)')
    parser.add_argument('--nav_bag', required=True, help='导航 bag 路径 (/agv_pose)')
    parser.add_argument('--output', default='nav_accuracy_result.png', help='输出图片路径')
    parser.add_argument('--rpe_delta', type=float, default=1.0, help='RPE 计算间隔 (秒)')
    args = parser.parse_args()

    print("=" * 60)
    print("导航精度评估")
    print("=" * 60)

    # 1. 读取真值轨迹 (从 TF)
    print(f"\n[1/4] 读取真值轨迹: {args.gt_bag}")
    gt_poses = read_tf_trajectory(args.gt_bag, 'map', 'base_footprint')
    print(f"  真值轨迹点数: {len(gt_poses)}")
    if not gt_poses:
        print("  错误: 未找到 map -> base_footprint 的 TF!")
        print("  尝试查找其他 TF 组合...")
        # 尝试 map -> odom
        gt_poses_odom = read_tf_trajectory(args.gt_bag, 'map', 'odom')
        print(f"  map -> odom 点数: {len(gt_poses_odom)}")
        return

    # 2. 读取估计轨迹 (从 /agv_pose)
    print(f"\n[2/4] 读取导航轨迹: {args.nav_bag}")
    est_poses = read_pose_topic(args.nav_bag, '/agv_pose')
    print(f"  导航轨迹点数: {len(est_poses)}")
    if not est_poses:
        print("  错误: 未找到 /agv_pose topic!")
        return

    # 3. 时间对齐
    print(f"\n[3/4] 时间对齐...")
    gt_aligned, est_aligned = align_trajectories(gt_poses, est_poses)
    print(f"  对齐后点数: {len(gt_aligned)}")
    if len(gt_aligned) < 10:
        print("  错误: 对齐后点数太少，两条轨迹可能时间不重叠!")
        return

    # 4. 计算精度指标
    print(f"\n[4/4] 计算精度指标...")
    ate = compute_ate(gt_aligned, est_aligned)
    rpe = compute_rpe(gt_aligned, est_aligned, delta=args.rpe_delta)

    print("\n" + "=" * 60)
    print("评估结果")
    print("=" * 60)
    print(f"\n--- ATE (绝对轨迹误差) ---")
    print(f"  RMSE:   {ate['rmse']:.4f} m")
    print(f"  Mean:   {ate['mean']:.4f} m")
    print(f"  Median: {ate['median']:.4f} m")
    print(f"  Max:    {ate['max']:.4f} m")
    print(f"  Std:    {ate['std']:.4f} m")

    print(f"\n--- RPE (相对位姿误差, Δ={args.rpe_delta}s) ---")
    print(f"  平移 RMSE: {rpe['rmse']:.4f} m")
    print(f"  平移 Mean: {rpe['mean']:.4f} m")
    print(f"  平移 Median: {rpe['median']:.4f} m")
    print(f"  旋转 RMSE: {rpe['rot_rmse_deg']:.2f}°")

    # 5. 画图
    print("\n绘制轨迹对比图...")
    plot_trajectories(gt_aligned, est_aligned, ate['errors'], args.output)


if __name__ == '__main__':
    main()
