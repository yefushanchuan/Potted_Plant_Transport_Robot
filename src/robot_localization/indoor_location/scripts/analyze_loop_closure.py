#!/usr/bin/env python3
"""回环闭合实验分析脚本

用法:
    python3 analyze_loop_closure.py <bag_dir>

从 rosbag 中提取 map->odom + /odom 合并轨迹，计算:
  - 回环闭合误差 (首尾帧距离)
  - ATE (每帧相对起点的偏差)
  - RPE (相邻帧间的相对位姿误差)

使用 ROS2 官方 API 解析消息，避免手动 CDR 解析的对齐问题。
"""

import argparse
import sys
import os
import numpy as np

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry


def normalize_angle(a):
    """角度归一化到 [-pi, pi]"""
    while a > np.pi:
        a -= 2 * np.pi
    while a < -np.pi:
        a += 2 * np.pi
    return a


def quat_to_yaw(qx, qy, qz, qw):
    """四元数转 yaw 角"""
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))


def is_valid_pose(x, y, z, qx, qy, qz, qw):
    """检查位姿数据是否有效"""
    vals = [x, y, z, qx, qy, qz, qw]
    if not all(np.isfinite(v) for v in vals):
        return False
    if abs(qx) > 1.0 or abs(qy) > 1.0 or abs(qz) > 1.0 or abs(qw) > 1.0:
        return False
    if abs(x) > 500 or abs(y) > 500 or abs(z) > 500:
        return False
    return True


def extract_trajectory(bag_path, map_frame="map"):
    """从 bag 中提取轨迹

    使用 map->odom TF + /odom 话题合并得到机器人实际位置。

    返回: [(stamp, x, y, yaw), ...]
    """
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    # 收集 map->odom TF 和 /odom 消息
    map_odom_list = []  # [(stamp, tx, ty, tz, qx, qy, qz, qw)]
    odom_list = []      # [(stamp, px, py, pz, qx, qy, qz, qw)]

    tf_count = 0
    odom_count = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == '/tf':
            msg = deserialize_message(data, TFMessage)
            for t in msg.transforms:
                if t.header.frame_id == map_frame and t.child_frame_id == 'odom':
                    stamp = t.header.stamp.sec + t.header.stamp.nanosec / 1e9
                    p = t.transform.translation
                    q = t.transform.rotation
                    if is_valid_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w):
                        map_odom_list.append((stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                    tf_count += 1

        elif topic == '/odom':
            msg = deserialize_message(data, Odometry)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            if is_valid_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w):
                odom_list.append((stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            odom_count += 1

    print(f"Extracted: {tf_count} map->odom TF, {len(map_odom_list)} valid")
    print(f"           {odom_count} /odom msgs, {len(odom_list)} valid")

    if not map_odom_list:
        print("Error: no map->odom TF found")
        return []
    if not odom_list:
        print("Error: no /odom topic found")
        return []

    # 合并 map->odom + odom->base_footprint
    map_odom_list.sort(key=lambda x: x[0])
    odom_list.sort(key=lambda x: x[0])

    print(f"\nUsing: map->odom TF + /odom topic")

    result = []
    j = 0
    for s, ox, oy, oz, oqx, oqy, oqz, oqw in odom_list:
        # 找时间最近的 map_odom 帧
        while j + 1 < len(map_odom_list) and abs(map_odom_list[j + 1][0] - s) < abs(map_odom_list[j][0] - s):
            j += 1
        if j < len(map_odom_list):
            ms, mx, my, mz, mqx, mqy, mqz, mqw = map_odom_list[j]
            try:
                yaw_m = quat_to_yaw(mqx, mqy, mqz, mqw)
                yaw_o = quat_to_yaw(oqx, oqy, oqz, oqw)
            except Exception:
                continue
            if not (np.isfinite(yaw_m) and np.isfinite(yaw_o)):
                continue

            # map->base = map->odom * odom->base (2D)
            cos_m, sin_m = np.cos(yaw_m), np.sin(yaw_m)
            bx = mx + cos_m * ox - sin_m * oy
            by = my + sin_m * ox + cos_m * oy
            byaw = normalize_angle(yaw_m + yaw_o)

            if np.isfinite(bx) and np.isfinite(by) and abs(bx) < 500 and abs(by) < 500:
                result.append((s, bx, by, byaw))

    print(f"  Merged {len(result)} valid poses")
    return result


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
    parser = argparse.ArgumentParser(description="回环闭合实验分析")
    parser.add_argument("bag_dir", help="bag 目录或 .db3 文件路径")
    parser.add_argument("--map-frame", default="map", help="地图坐标系 (default: map)")
    parser.add_argument("--rpe-delta", type=int, default=1, help="RPE 帧间隔 (default: 1)")
    args = parser.parse_args()

    # 找 bag 目录
    bag_path = args.bag_dir
    if os.path.isfile(bag_path) and bag_path.endswith('.db3'):
        bag_path = os.path.dirname(bag_path)

    if not os.path.exists(bag_path):
        print(f"Error: bag path '{bag_path}' does not exist")
        sys.exit(1)

    print(f"Analyzing bag: {bag_path}")
    print()

    # 提取轨迹
    poses = extract_trajectory(bag_path, args.map_frame)

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
    traj_file = bag_path.rstrip('/') + '_trajectory.csv'
    with open(traj_file, 'w') as f:
        f.write("timestamp,x,y,yaw\n")
        for stamp, x, y, yaw in poses:
            f.write(f"{stamp:.9f},{x:.6f},{y:.6f},{yaw:.6f}\n")
    print(f"Trajectory saved to: {traj_file}")


if __name__ == "__main__":
    main()
