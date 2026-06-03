#!/usr/bin/env python3
"""从 rosbag 中提取 EKF 和全局定位轨迹并绘图"""
import sqlite3
import struct
import math
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

BAG_PATH = Path(__file__).parent / 'trajectory_bag_0.db3'

def deserialize_pose_stamped(data: bytes):
    """反序列化 geometry_msgs/PoseStamped（手动解析 CDR）"""
    # CDR header: encapsulation scheme(4) + options(4)
    off = 8
    # stamp
    sec = struct.unpack_from('<i', data, off)[0]; off += 4
    nsec = struct.unpack_from('<I', data, off)[0]; off += 4
    # frame_id (string)
    slen = struct.unpack_from('<I', data, off)[0]; off += 4
    frame_id = data[off:off+slen-1].decode('utf-8', errors='replace'); off += slen
    # pose.position
    px = struct.unpack_from('<d', data, off)[0]; off += 8
    py = struct.unpack_from('<d', data, off)[0]; off += 8
    pz = struct.unpack_from('<d', data, off)[0]; off += 8
    # pose.orientation
    ox = struct.unpack_from('<d', data, off)[0]; off += 8
    oy = struct.unpack_from('<d', data, off)[0]; off += 8
    oz = struct.unpack_from('<d', data, off)[0]; off += 8
    ow = struct.unpack_from('<d', data, off)[0]; off += 8
    t = sec + nsec * 1e-9
    yaw = math.atan2(2.0 * (ow * oz + ox * oy), 1.0 - 2.0 * (oy * oy + oz * oz))
    return t, px, py, yaw

def deserialize_tf_message(data: bytes):
    """反序列化 tf2_msgs/TFMessage，返回 [(t, x, y, yaw), ...]"""
    results = []
    off = 8  # CDR header
    # array of TransformStamped
    n_transforms = struct.unpack_from('<I', data, off)[0]; off += 4
    for _ in range(n_transforms):
        # stamp
        sec = struct.unpack_from('<i', data, off)[0]; off += 4
        nsec = struct.unpack_from('<U', data, off)[0]; off += 4
        # child_frame_id (string)
        slen = struct.unpack_from('<I', data, off)[0]; off += 4
        child_frame = data[off:off+slen-1].decode('utf-8', errors='replace'); off += slen
        # header.frame_id (string)
        slen = struct.unpack_from('<I', data, off)[0]; off += 4
        parent_frame = data[off:off+slen-1].decode('utf-8', errors='replace'); off += slen
        # transform.translation
        tx = struct.unpack_from('<d', data, off)[0]; off += 8
        ty = struct.unpack_from('<d', data, off)[0]; off += 8
        tz = struct.unpack_from('<d', data, off)[0]; off += 8
        # transform.rotation
        rx = struct.unpack_from('<d', data, off)[0]; off += 8
        ry = struct.unpack_from('<d', data, off)[0]; off += 8
        rz = struct.unpack_from('<d', data, off)[0]; off += 8
        rw = struct.unpack_from('<d', data, off)[0]; off += 8
        t = sec + nsec * 1e-9
        yaw = math.atan2(2.0 * (rw * rz + rx * ry), 1.0 - 2.0 * (ry * ry + rz * rz))
        results.append((parent_frame, child_frame, t, tx, ty, yaw))
    return results

def main():
    conn = sqlite3.connect(str(BAG_PATH))
    cur = conn.cursor()

    # 读取全局定位轨迹
    global_traj = []
    cur.execute('SELECT data FROM messages WHERE topic_id=1 ORDER BY timestamp')
    for (data,) in cur:
        try:
            t, px, py, yaw = deserialize_pose_stamped(data)
            global_traj.append((t, px, py, yaw))
        except Exception:
            pass

    # 读取 TF，提取 odom→base_footprint（EKF 轨迹）
    ekf_traj = []
    cur.execute('SELECT data FROM messages WHERE topic_id=3 ORDER BY timestamp')
    for (data,) in cur:
        try:
            tfs = deserialize_tf_message(data)
            for parent, child, t, tx, ty, yaw in tfs:
                if 'base' in child and 'odom' in parent:
                    ekf_traj.append((t, tx, ty, yaw))
        except Exception:
            pass

    conn.close()

    print(f'全局定位轨迹: {len(global_traj)} 点')
    print(f'EKF 轨迹: {len(ekf_traj)} 点')

    if not global_traj and not ekf_traj:
        print('没有轨迹数据')
        return

    # 绘图
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))

    if ekf_traj:
        ekf_x = [p[1] for p in ekf_traj]
        ekf_y = [p[2] for p in ekf_traj]
        ax.plot(ekf_x, ekf_y, 'b-', linewidth=1.5, label=f'EKF (odom) [{len(ekf_traj)} pts]', alpha=0.8)
        ax.plot(ekf_x[0], ekf_y[0], 'bs', markersize=10)
        ax.plot(ekf_x[-1], ekf_y[-1], 'b^', markersize=10)

    if global_traj:
        gx = [p[1] for p in global_traj]
        gy = [p[2] for p in global_traj]
        ax.plot(gx, gy, 'r-', linewidth=1.5, label=f'Global (map) [{len(global_traj)} pts]', alpha=0.8)
        ax.plot(gx[0], gy[0], 'rs', markersize=10)
        ax.plot(gx[-1], gy[-1], 'r^', markersize=10)

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('Trajectory Comparison: EKF vs Global Localization', fontsize=14)
    ax.legend(fontsize=11)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    out_path = Path(__file__).parent / 'trajectory_comparison.png'
    plt.tight_layout()
    plt.savefig(str(out_path), dpi=150, bbox_inches='tight')
    print(f'已保存: {out_path}')
    plt.close()

if __name__ == '__main__':
    main()
