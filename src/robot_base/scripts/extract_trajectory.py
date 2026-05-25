#!/usr/bin/env python3
"""从 rosbag2 提取轨迹，输出 TUM 格式（timestamp tx ty tz qx qy qz qw）"""

import sqlite3
import struct
import sys
import os


def parse_odom_cdr(data):
    """Parse nav_msgs/msg/Odometry CDR LE, return (stamp_sec, px,py,pz,qx,qy,qz,qw)"""
    o = 4  # skip CDR header
    # Header.stamp
    stamp_sec = struct.unpack_from('<i', data, o)[0]; o += 4
    stamp_nsec = struct.unpack_from('<I', data, o)[0]; o += 4
    stamp = stamp_sec + stamp_nsec / 1e9
    # Header.frame_id
    flen = struct.unpack_from('<I', data, o)[0]; o += 4
    o += flen; o += (4 - o % 4) % 4
    # child_frame_id
    clen = struct.unpack_from('<I', data, o)[0]; o += 4
    o += clen; o += (4 - o % 4) % 4
    # Pose
    px = struct.unpack_from('<d', data, o)[0]; o += 8
    py = struct.unpack_from('<d', data, o)[0]; o += 8
    pz = struct.unpack_from('<d', data, o)[0]; o += 8
    qx = struct.unpack_from('<d', data, o)[0]; o += 8
    qy = struct.unpack_from('<d', data, o)[0]; o += 8
    qz = struct.unpack_from('<d', data, o)[0]; o += 8
    qw = struct.unpack_from('<d', data, o)[0]; o += 8
    return stamp, px, py, pz, qx, qy, qz, qw


def extract_trajectory(bag_db3, topic_name, output_file, t_start=None, t_end=None):
    """从 bag 的 db3 文件提取指定 topic 的轨迹
    t_start, t_end: 相对于 bag 起始时间的秒数，用于截断
    """
    conn = sqlite3.connect(bag_db3)
    cur = conn.cursor()

    # 获取 topic id
    cur.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
    row = cur.fetchone()
    if row is None:
        print(f"Error: topic '{topic_name}' not found in {bag_db3}")
        conn.close()
        return False
    topic_id = row[0]

    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (topic_id,))
    rows = cur.fetchall()

    # 用第一条消息的 header.stamp 作为基准
    first_stamp, *_ = parse_odom_cdr(rows[0][1])
    t0 = first_stamp
    t_start_abs = t0 + t_start if t_start is not None else None
    t_end_abs = t0 + t_end if t_end is not None else None

    count = 0
    with open(output_file, 'w') as f:
        # TUM 格式: header.stamp tx ty tz qx qy qz qw
        for ts_ns, data in rows:
            stamp, px, py, pz, qx, qy, qz, qw = parse_odom_cdr(data)
            if t_start_abs is not None and stamp < t_start_abs:
                continue
            if t_end_abs is not None and stamp > t_end_abs:
                break
            f.write(f"{stamp:.9f} {px:.6f} {py:.6f} {pz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
            count += 1

    conn.close()
    range_str = ""
    if t_start is not None or t_end is not None:
        range_str = f" [{t_start or 0}s ~ {t_end or 'end'}s]"
    print(f"Extracted {count} poses from '{topic_name}' -> {output_file}{range_str}")
    return True


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="从 rosbag2 提取轨迹 (TUM 格式)")
    parser.add_argument("bag_dir", help="bag 目录或 .db3 文件路径")
    parser.add_argument("topic", help="odom topic 名称")
    parser.add_argument("output", help="输出文件路径")
    parser.add_argument("--start", type=float, default=None, help="截断起点（相对 bag 起始的秒数）")
    parser.add_argument("--end", type=float, default=None, help="截断终点（相对 bag 起始的秒数）")
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

    extract_trajectory(db3_path, args.topic, args.output, args.start, args.end)
