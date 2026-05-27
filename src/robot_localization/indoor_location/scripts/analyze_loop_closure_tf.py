#!/usr/bin/env python3
"""回环闭合实验分析脚本 (使用 odom->base_footprint TF)

用法:
    python3 analyze_loop_closure_tf.py <bag_dir>

从 rosbag 中提取 map->odom + odom->base_footprint TF 合并轨迹，计算:
  - 回环闭合误差 (首尾帧距离)
  - ATE (每帧相对起点的偏差)
  - RPE (相邻帧间的相对位姿误差)
"""

import argparse
import sys
import os
import sqlite3
import struct
import numpy as np


def normalize_angle(a):
    """角度归一化到 [-pi, pi]"""
    while a > np.pi:
        a -= 2 * np.pi
    while a < -np.pi:
        a += 2 * np.pi
    return a


def read_cdr_string(data, o):
    """读取 CDR 字符串，返回 (字符串, 新偏移量)"""
    slen = struct.unpack_from('<I', data, o)[0]; o += 4
    s = data[o:o + slen].decode('utf-8', errors='ignore').rstrip('\x00')
    o += slen
    o += (4 - o % 4) % 4
    return s, o


def is_valid_tf(tx, ty, tz, qx, qy, qz, qw):
    """检查 TF 数据是否有效"""
    vals = [tx, ty, tz, qx, qy, qz, qw]
    if not all(np.isfinite(v) for v in vals):
        return False
    if abs(qx) > 1.0 or abs(qy) > 1.0 or abs(qz) > 1.0 or abs(qw) > 1.0:
        return False
    if abs(tx) > 500 or abs(ty) > 500 or abs(tz) > 500:
        return False
    return True


def quat_to_yaw(qx, qy, qz, qw):
    """四元数转 yaw 角"""
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))


def parse_tf_msg_cdr(data):
    """解析 tf2_msgs/TFMessage CDR LE
    返回 [(frame_id, child_frame_id, stamp, tx, ty, tz, qx, qy, qz, qw), ...]
    """
    o = 4  # CDR header
    seq_len = struct.unpack_from('<I', data, o)[0]; o += 4

    results = []
    for _ in range(seq_len):
        try:
            stamp_sec = struct.unpack_from('<i', data, o)[0]; o += 4
            stamp_nsec = struct.unpack_from('<I', data, o)[0]; o += 4
            stamp = stamp_sec + stamp_nsec / 1e9
            frame_id, o = read_cdr_string(data, o)
            child_frame_id, o = read_cdr_string(data, o)
            tx = struct.unpack_from('<d', data, o)[0]; o += 8
            ty = struct.unpack_from('<d', data, o)[0]; o += 8
            tz = struct.unpack_from('<d', data, o)[0]; o += 8
            qx = struct.unpack_from('<d', data, o)[0]; o += 8
            qy = struct.unpack_from('<d', data, o)[0]; o += 8
            qz = struct.unpack_from('<d', data, o)[0]; o += 8
            qw = struct.unpack_from('<d', data, o)[0]; o += 8
            results.append((frame_id, child_frame_id, stamp, tx, ty, tz, qx, qy, qz, qw))
        except Exception:
            break
    return results


def compute_ate(poses):
    """计算 ATE (Absolute Trajectory Error)，以第一帧为原点"""
    if len(poses) < 2:
        return []
    x0, y0 = poses[0][1], poses[0][2]
    errors = []
    for _, x, y, _ in poses:
        dist = np.sqrt((x - x0)**2 + (y - y0)**2)
        if np.isfinite(dist):
            errors.append(dist)
    return errors


def compute_rpe(poses, delta=1):
    """计算 RPE (Relative Pose Error)，相邻 delta 帧间的相对误差"""
    if len(poses) < delta + 1:
        return []
    errors = []
    for i in range(len(poses) - delta):
        _, x1, y1, yaw1 = poses[i]
        _, x2, y2, yaw2 = poses[i + delta]
        dx = x2 - x1
        dy = y2 - y1
        cos1, sin1 = np.cos(yaw1), np.sin(yaw1)
        local_dx = cos1 * dx + sin1 * dy
        local_dy = -sin1 * dx + cos1 * dy
        trans_err = np.sqrt(local_dx**2 + local_dy**2)
        ang_err = abs(normalize_angle(yaw2 - yaw1))
        if np.isfinite(trans_err) and np.isfinite(ang_err):
            errors.append((trans_err, ang_err))
    return errors


def print_stats(name, values, unit="m"):
    """打印统计信息"""
    if not values:
        print(f"  {name}:  No data")
        return
    arr = np.array(values)
    rmse = np.sqrt(np.mean(arr**2))
    mean = np.mean(arr)
    median = np.median(arr)
    max_val = np.max(arr)
    print(f"  {name}:")
    print(f"    RMSE:   {rmse:.4f} {unit}")
    print(f"    Mean:   {mean:.4f} {unit}")
    print(f"    Median: {median:.4f} {unit}")
    print(f"    Max:    {max_val:.4f} {unit}")


def main():
    parser = argparse.ArgumentParser(description="回环闭合实验分析 (使用 odom->base_footprint TF)")
    parser.add_argument("bag_dir", help="bag 目录或 .db3 文件路径")
    parser.add_argument("--map-frame", default="map", help="地图坐标系 (default: map)")
    parser.add_argument("--base-frame", default="base_footprint", help="机器人坐标系 (default: base_footprint)")
    parser.add_argument("--rpe-delta", type=int, default=1, help="RPE 帧间隔 (default: 1)")
    args = parser.parse_args()

    # 找 db3 文件
    db3_path = None
    if os.path.isfile(args.bag_dir) and args.bag_dir.endswith('.db3'):
        db3_path = args.bag_dir
    else:
        for f in os.listdir(args.bag_dir):
            if f.endswith('.db3'):
                db3_path = os.path.join(args.bag_dir, f)
                break

    if db3_path is None:
        print(f"Error: no .db3 file found in {args.bag_dir}")
        sys.exit(1)

    print(f"Analyzing bag: {db3_path}")
    print()

    # 提取 TF
    conn = sqlite3.connect(db3_path)
    cur = conn.cursor()
    cur.execute("SELECT id, name FROM topics WHERE name LIKE '%tf%'")
    tf_topics = cur.fetchall()

    tf_by_pair = {}
    total_raw = 0
    total_filtered = 0
    for topic_id, topic_name in tf_topics:
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (topic_id,))
        for _, data in cur.fetchall():
            try:
                transforms = parse_tf_msg_cdr(data)
                for frame_id, child_frame_id, stamp, tx, ty, tz, qx, qy, qz, qw in transforms:
                    total_raw += 1
                    if not is_valid_tf(tx, ty, tz, qx, qy, qz, qw):
                        total_filtered += 1
                        continue
                    pair = (frame_id, child_frame_id)
                    if pair not in tf_by_pair:
                        tf_by_pair[pair] = []
                    tf_by_pair[pair].append((stamp, tx, ty, tz, qx, qy, qz, qw))
            except Exception:
                continue
    conn.close()

    print(f"TF parsing: {total_raw} raw, {total_filtered} filtered, {total_raw - total_filtered} valid")
    for (f, c) in sorted(tf_by_pair.keys()):
        print(f"  {f} -> {c}: {len(tf_by_pair[(f, c)])} frames")

    # 合并 map->odom + odom->base_footprint
    map_to_odom = (args.map_frame, 'odom')
    odom_to_base = ('odom', args.base_frame)

    if map_to_odom not in tf_by_pair:
        print(f"Error: {args.map_frame}->odom TF not found")
        sys.exit(1)
    if odom_to_base not in tf_by_pair:
        print(f"Error: odom->{args.base_frame} TF not found")
        sys.exit(1)

    print(f"\nUsing chained TF: {args.map_frame} -> odom -> {args.base_frame}")
    map_odom = sorted(tf_by_pair[map_to_odom], key=lambda x: x[0])
    odom_base = sorted(tf_by_pair[odom_to_base], key=lambda x: x[0])

    poses = []
    j = 0
    for s, ox, oy, oz, oqx, oqy, oqz, oqw in odom_base:
        while j + 1 < len(map_odom) and abs(map_odom[j + 1][0] - s) < abs(map_odom[j][0] - s):
            j += 1
        if j < len(map_odom):
            ms, mx, my, mz, mqx, mqy, mqz, mqw = map_odom[j]
            try:
                yaw_m = quat_to_yaw(mqx, mqy, mqz, mqw)
                yaw_o = quat_to_yaw(oqx, oqy, oqz, oqw)
            except Exception:
                continue
            if not (np.isfinite(yaw_m) and np.isfinite(yaw_o)):
                continue
            cos_m, sin_m = np.cos(yaw_m), np.sin(yaw_m)
            bx = mx + cos_m * ox - sin_m * oy
            by = my + sin_m * ox + cos_m * oy
            byaw = normalize_angle(yaw_m + yaw_o)
            if np.isfinite(bx) and np.isfinite(by) and abs(bx) < 500 and abs(by) < 500:
                poses.append((s, bx, by, byaw))

    print(f"  Merged {len(poses)} valid poses from {len(map_odom)} map->odom + {len(odom_base)} odom->base")

    if len(poses) < 2:
        print("Error: insufficient poses extracted")
        sys.exit(1)

    print(f"Duration: {poses[-1][0] - poses[0][0]:.1f} seconds")
    print()

    # 计算回环闭合误差
    x0, y0, yaw0 = float(poses[0][1]), float(poses[0][2]), float(poses[0][3])
    x_end, y_end, yaw_end = float(poses[-1][1]), float(poses[-1][2]), float(poses[-1][3])
    loop_err_xy = np.sqrt((x_end - x0)**2 + (y_end - y0)**2)
    loop_err_yaw = abs(normalize_angle(yaw_end - yaw0))

    print("=" * 50)
    print("  Loop Closure Error")
    print("=" * 50)
    print(f"  XY:   {loop_err_xy:.4f} m ({loop_err_xy * 100:.1f} cm)")
    print(f"  Yaw:  {np.degrees(loop_err_yaw):.2f} deg")
    print()

    # 计算 ATE
    ate_errors = compute_ate(poses)
    print("=" * 50)
    print("  ATE (Absolute Trajectory Error)")
    print("=" * 50)
    print_stats("ATE", ate_errors)
    print()

    # 计算 RPE
    rpe_results = compute_rpe(poses, delta=args.rpe_delta)
    rpe_trans = [e[0] for e in rpe_results]
    rpe_ang = [e[1] for e in rpe_results]

    print("=" * 50)
    print(f"  RPE (Relative Pose Error, delta={args.rpe_delta})")
    print("=" * 50)
    print_stats("Translation", rpe_trans)
    print_stats("Rotation", np.degrees(rpe_ang).tolist(), unit="deg")
    print()

    # 输出轨迹
    traj_file = args.bag_dir.rstrip('/') + '_trajectory_tf.csv'
    with open(traj_file, 'w') as f:
        f.write("timestamp,x,y,yaw\n")
        for stamp, x, y, yaw in poses:
            f.write(f"{stamp:.9f},{x:.6f},{y:.6f},{yaw:.6f}\n")
    print(f"Trajectory saved to: {traj_file}")


if __name__ == "__main__":
    main()
