#!/usr/bin/env python3
"""
导航精度评估脚本

用法:
  python3 eval_nav_accuracy.py --gt_bag nav_gt_low --nav_bag nav_online_low --output result_low.png

从两个 rosbag 中提取轨迹:
  - 真值: TF map -> base_footprint (来自 FAST-LIO2 + BA)
  - 待评估: /agv_pose (来自定位节点)

输出: ATE/RPE 指标 + 轨迹对比图
"""

import argparse
import numpy as np
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import register_types, get_types_from_msg
from rosbags.highlevel import AnyReader


def read_pose_topic(bag_path, topic_name):
    """从 bag 中读取 PoseStamped topic，返回 [(timestamp_sec, x, y, yaw), ...]"""
    poses = []
    with AnyReader([Path(bag_path)]) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic_name:
                msg = reader.deserialize(rawdata, connection.msgtype)
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                x = msg.pose.position.x
                y = msg.pose.position.y
                q = msg.pose.orientation
                # quaternion to yaw
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = np.arctan2(siny_cosp, cosy_cosp)
                poses.append((t, x, y, yaw))
    return poses


def quat_to_yaw(q):
    """Convert quaternion to yaw angle"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


def compose_tf(t1, t2):
    """Compose two transforms: result = t1 * t2 (t1 applied first, then t2)"""
    # For 2D (x, y, yaw):
    # result_x = t1_x + cos(t1_yaw) * t2_x - sin(t1_yaw) * t2_y
    # result_y = t1_y + sin(t1_yaw) * t2_x + cos(t1_yaw) * t2_y
    # result_yaw = t1_yaw + t2_yaw
    x1, y1, yaw1 = t1
    x2, y2, yaw2 = t2
    cos_yaw1 = np.cos(yaw1)
    sin_yaw1 = np.sin(yaw1)
    x = x1 + cos_yaw1 * x2 - sin_yaw1 * y2
    y = y1 + sin_yaw1 * x2 + cos_yaw1 * y2
    yaw = yaw1 + yaw2
    return (x, y, yaw)


def read_tf_trajectory(bag_path, parent_frame='map', child_frame='base_footprint'):
    """从 bag 中读取 TF，如果直接 TF 不存在则尝试组合"""
    # First try direct TF
    poses = []
    tf_map_odom = {}  # timestamp -> (x, y, yaw)
    tf_odom_bf = {}   # timestamp -> (x, y, yaw)

    with AnyReader([Path(bag_path)]) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/tf':
                msg = reader.deserialize(rawdata, connection.msgtype)
                for transform in msg.transforms:
                    t = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                    x = transform.transform.translation.x
                    y = transform.transform.translation.y
                    yaw = quat_to_yaw(transform.transform.rotation)

                    if transform.header.frame_id == parent_frame and \
                       transform.child_frame_id == child_frame:
                        poses.append((t, x, y, yaw))

                    if transform.header.frame_id == 'map' and \
                       transform.child_frame_id == 'odom':
                        tf_map_odom[t] = (x, y, yaw)

                    if transform.header.frame_id == 'odom' and \
                       transform.child_frame_id == 'base_footprint':
                        tf_odom_bf[t] = (x, y, yaw)

    # If direct TF found, return it
    if poses:
        poses.sort(key=lambda p: p[0])
        return poses

    # Otherwise, compose map->odom and odom->base_footprint
    print(f"  Direct TF {parent_frame}->{child_frame} not found, composing from map->odom + odom->base_footprint")
    print(f"  map->odom: {len(tf_map_odom)} samples, odom->base_footprint: {len(tf_odom_bf)} samples")

    if not tf_map_odom or not tf_odom_bf:
        return []

    # For each odom->base_footprint timestamp, find closest map->odom
    odom_times = sorted(tf_odom_bf.keys())
    odom_map_times = sorted(tf_map_odom.keys())

    for odom_t in odom_times:
        # Find closest map->odom timestamp
        idx = np.searchsorted(odom_map_times, odom_t)
        if idx == 0:
            map_t = odom_map_times[0]
        elif idx >= len(odom_map_times):
            map_t = odom_map_times[-1]
        else:
            # Pick closer one
            t0 = odom_map_times[idx - 1]
            t1 = odom_map_times[idx]
            map_t = t0 if abs(odom_t - t0) <= abs(odom_t - t1) else t1

        # Compose
        map_odom = tf_map_odom[map_t]
        odom_bf = tf_odom_bf[odom_t]
        composed = compose_tf(map_odom, odom_bf)
        poses.append((odom_t, composed[0], composed[1], composed[2]))

    poses.sort(key=lambda p: p[0])
    return poses


def interpolate_trajectory(poses, target_times):
    """对轨迹进行时间插值"""
    if not poses:
        return []
    result = []
    times = [p[0] for p in poses]
    for t in target_times:
        if t <= times[0]:
            result.append(poses[0])
        elif t >= times[-1]:
            result.append(poses[-1])
        else:
            # binary search
            lo, hi = 0, len(times) - 1
            while hi - lo > 1:
                mid = (lo + hi) // 2
                if times[mid] <= t:
                    lo = mid
                else:
                    hi = mid
            # linear interpolation
            t0, t1 = times[lo], times[hi]
            alpha = (t - t0) / (t1 - t0) if t1 != t0 else 0
            x = poses[lo][1] + alpha * (poses[hi][1] - poses[lo][1])
            y = poses[lo][2] + alpha * (poses[hi][2] - poses[lo][2])
            # angle interpolation
            a0, a1 = poses[lo][3], poses[hi][3]
            diff = a1 - a0
            if diff > np.pi:
                diff -= 2 * np.pi
            elif diff < -np.pi:
                diff += 2 * np.pi
            yaw = a0 + alpha * diff
            result.append((t, x, y, yaw))
    return result


def umeyama_alignment(src, dst):
    """Umeyama alignment: find rotation R and translation t such that dst ≈ R * src + t.
    src, dst: Nx2 arrays of (x, y) positions.
    Returns: R (2x2), t (2,), scale (float)
    """
    src_mean = np.mean(src, axis=0)
    dst_mean = np.mean(dst, axis=0)
    src_centered = src - src_mean
    dst_centered = dst - dst_mean
    # SVD
    H = src_centered.T @ dst_centered / len(src)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    # handle reflection
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = dst_mean - R @ src_mean
    return R, t


def compute_ate(gt_poses, nav_poses):
    """计算 Absolute Trajectory Error (with trajectory alignment)"""
    if not gt_poses or not nav_poses:
        return None, None
    # find common time range
    gt_times = [p[0] for p in gt_poses]
    nav_times = [p[0] for p in nav_poses]
    t_start = max(gt_times[0], nav_times[0])
    t_end = min(gt_times[-1], nav_times[-1])
    if t_start >= t_end:
        return None, None
    # sample at nav times
    nav_filtered = [p for p in nav_poses if t_start <= p[0] <= t_end]
    if len(nav_filtered) < 2:
        return None, None
    target_times = [p[0] for p in nav_filtered]
    gt_interp = interpolate_trajectory(gt_poses, target_times)
    if len(gt_interp) < 3:
        return None, None
    # align trajectories using Umeyama
    gt_xy = np.array([[p[1], p[2]] for p in gt_interp])
    nav_xy = np.array([[p[1], p[2]] for p in nav_filtered])
    R, t = umeyama_alignment(gt_xy, nav_xy)
    # apply alignment
    gt_aligned_xy = (R @ gt_xy.T).T + t
    # compute rotation angle for yaw alignment
    angle = np.arctan2(R[1, 0], R[0, 0])
    # compute errors
    pos_errors = []
    yaw_errors = []
    for i, (gt, nav) in enumerate(zip(gt_interp, nav_filtered)):
        dx = gt_aligned_xy[i, 0] - nav[1]
        dy = gt_aligned_xy[i, 1] - nav[2]
        pos_errors.append(np.sqrt(dx*dx + dy*dy))
        gt_yaw = gt[3] + angle
        dyaw = gt_yaw - nav[3]
        if dyaw > np.pi:
            dyaw -= 2 * np.pi
        elif dyaw < -np.pi:
            dyaw += 2 * np.pi
        yaw_errors.append(abs(dyaw))
    return pos_errors, yaw_errors


def compute_rpe(gt_poses, nav_poses, delta=1.0):
    """计算 Relative Pose Error (每隔 delta 秒, rotation-invariant)"""
    if not gt_poses or not nav_poses:
        return None
    gt_times = [p[0] for p in gt_poses]
    nav_times = [p[0] for p in nav_poses]
    t_start = max(gt_times[0], nav_times[0])
    t_end = min(gt_times[-1], nav_times[-1])
    if t_start >= t_end:
        return None
    # sample pairs
    pair_times = []
    t = t_start
    while t + delta <= t_end:
        pair_times.append((t, t + delta))
        t += delta
    if not pair_times:
        return None
    # interpolate
    all_times = sorted(set([t for pair in pair_times for t in pair]))
    gt_interp = {p[0]: p for p in interpolate_trajectory(gt_poses, all_times)}
    nav_interp = {p[0]: p for p in interpolate_trajectory(nav_poses, all_times)}
    rpe_errors = []
    for t1, t2 in pair_times:
        if t1 in gt_interp and t2 in gt_interp and t1 in nav_interp and t2 in nav_interp:
            gt1, gt2 = gt_interp[t1], gt_interp[t2]
            nav1, nav2 = nav_interp[t1], nav_interp[t2]
            # relative position error (rotation-invariant)
            gt_dx = gt2[1] - gt1[1]
            gt_dy = gt2[2] - gt1[2]
            nav_dx = nav2[1] - nav1[1]
            nav_dy = nav2[2] - nav1[2]
            err = np.sqrt((gt_dx - nav_dx)**2 + (gt_dy - nav_dy)**2)
            rpe_errors.append(err)
    return rpe_errors


def main():
    parser = argparse.ArgumentParser(description='Navigation Accuracy Evaluation')
    parser.add_argument('--gt_bag', required=True, help='Ground truth bag path')
    parser.add_argument('--nav_bag', required=True, help='Navigation bag path')
    parser.add_argument('--output', default='result.png', help='Output image path')
    args = parser.parse_args()

    print(f"Reading ground truth TF from {args.gt_bag}...")
    gt_poses = read_tf_trajectory(args.gt_bag)
    print(f"  Found {len(gt_poses)} TF samples")

    print(f"Reading navigation pose from {args.nav_bag}...")
    nav_poses = read_pose_topic(args.nav_bag, '/agv_pose')
    print(f"  Found {len(nav_poses)} pose samples")

    if not gt_poses:
        print("ERROR: No ground truth TF found!")
        return
    if not nav_poses:
        print("ERROR: No navigation pose found!")
        return

    print("\nComputing ATE...")
    pos_errors, yaw_errors = compute_ate(gt_poses, nav_poses)
    if pos_errors:
        print(f"  Position ATE: mean={np.mean(pos_errors):.3f}m, std={np.std(pos_errors):.3f}m, max={np.max(pos_errors):.3f}m")
        print(f"  Yaw ATE: mean={np.degrees(np.mean(yaw_errors)):.2f}°, std={np.degrees(np.std(yaw_errors)):.2f}°")
    else:
        print("  ERROR: Could not compute ATE (no overlapping time range)")

    print("\nComputing RPE (delta=1.0s)...")
    rpe_errors = compute_rpe(gt_poses, nav_poses, delta=1.0)
    if rpe_errors:
        print(f"  Position RPE: mean={np.mean(rpe_errors):.3f}m, std={np.std(rpe_errors):.3f}m, max={np.max(rpe_errors):.3f}m")
    else:
        print("  ERROR: Could not compute RPE")

    # Summary
    print("\n" + "="*50)
    print("SUMMARY")
    print("="*50)
    if pos_errors:
        print(f"ATE (position): {np.mean(pos_errors):.3f} ± {np.std(pos_errors):.3f} m")
        print(f"ATE (yaw): {np.degrees(np.mean(yaw_errors)):.2f} ± {np.degrees(np.std(yaw_errors)):.2f} °")
    if rpe_errors:
        print(f"RPE (position, delta=1s): {np.mean(rpe_errors):.3f} ± {np.std(rpe_errors):.3f} m")
    print(f"\nOutput: {args.output}")


if __name__ == '__main__':
    main()
