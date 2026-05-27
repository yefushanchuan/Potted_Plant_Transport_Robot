#!/usr/bin/env python3
"""重定位实验数据记录器

订阅话题:
  /localization_status  (std_msgs/Int8)    - 0=正常, 1=迷失
  /agv_pose             (geometry_msgs/PoseStamped) - 主节点位姿
  /icp_pose             (geometry_msgs/PoseWithCovarianceStamped) - 副节点重定位结果
  /loc_cmd              (std_msgs/String)  - 记录 force_lost 触发时刻

输出:
  CSV 文件: 时间戳, 状态, x, y, yaw, fitness(如有), 是否收到icp_pose
  实验结束自动统计: 成功率, 耗时, 精度

用法:
  ros2 run indoor_location reloc_experiment_recorder.py [--output FILE]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Int8, String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import csv
import os
import time
import math
import argparse
from datetime import datetime
from collections import deque


class RelocRecorder(Node):
    def __init__(self, output_path: str):
        super().__init__('reloc_experiment_recorder')

        # 状态
        self.current_status = 0  # 0=正常, 1=迷失
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_icp_pose = False  # 本帧是否收到过 icp_pose

        # 实验事件追踪
        self.events = []  # list of dict: {type, start_time, end_time, pre_x, pre_y, ...}
        self.lost_start_time = None
        self.lost_pre_x = 0.0
        self.lost_pre_y = 0.0
        self.lost_pre_yaw = 0.0
        self.icp_received_during_lost = False

        # CSV 记录
        self.output_path = output_path
        self.csv_file = open(output_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'wall_time', 'status', 'x', 'y', 'yaw', 'has_icp_pose'
        ])

        # 订阅
        qos = QoSProfile(depth=10)

        self.create_subscription(Int8, '/localization_status', self._status_cb, qos)
        self.create_subscription(PoseStamped, '/agv_pose', self._pose_cb, qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/icp_pose', self._icp_pose_cb, qos)
        self.create_subscription(String, '/loc_cmd', self._cmd_cb, qos)

        self.get_logger().info(f'记录器已启动, 输出文件: {output_path}')
        self.get_logger().info('等待实验开始... 发送 force_lost 命令触发迷失')

    # ---- 回调 ----

    def _status_cb(self, msg: Int8):
        prev_status = self.current_status
        self.current_status = msg.data
        self._write_row()

        # 检测迷失开始
        if prev_status == 0 and self.current_status == 1:
            self.lost_start_time = time.time()
            self.lost_pre_x = self.current_x
            self.lost_pre_y = self.current_y
            self.lost_pre_yaw = self.current_yaw
            self.icp_received_during_lost = False
            self.get_logger().warn(
                f'迷失开始 | 位姿=({self.current_x:.2f}, {self.current_y:.2f})')

        # 检测恢复
        if prev_status == 1 and self.current_status == 0:
            elapsed = time.time() - self.lost_start_time if self.lost_start_time else 0
            dx = self.current_x - self.lost_pre_x
            dy = self.current_y - self.lost_pre_y
            err = math.sqrt(dx * dx + dy * dy)
            method = 'icp辅助' if self.icp_received_during_lost else '主节点自恢复'

            event = {
                'method': method,
                'elapsed_sec': elapsed,
                'pre_x': self.lost_pre_x,
                'pre_y': self.lost_pre_y,
                'post_x': self.current_x,
                'post_y': self.current_y,
                'pos_error': err,
            }
            self.events.append(event)

            self.get_logger().info(
                f'恢复成功! 方法={method} 耗时={elapsed:.2f}s '
                f'精度={err:.3f}m '
                f'({self.lost_pre_x:.2f},{self.lost_pre_y:.2f}) -> '
                f'({self.current_x:.2f},{self.current_y:.2f})')

            self.lost_start_time = None

    def _pose_cb(self, msg: PoseStamped):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def _icp_pose_cb(self, msg: PoseWithCovarianceStamped):
        self.has_icp_pose = True
        if self.current_status == 1:
            self.icp_received_during_lost = True
            self.get_logger().info(
                f'收到副节点重定位结果: '
                f'({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})')

    def _cmd_cb(self, msg: String):
        if msg.data == 'force_lost':
            self.get_logger().warn('收到 force_lost 命令, 记录实验标记')
            self.csv_writer.writerow([
                '', datetime.now().isoformat(), 'FORCE_LOST',
                self.current_x, self.current_y, self.current_yaw, ''
            ])

    # ---- 记录 ----

    def _write_row(self):
        self.csv_writer.writerow([
            time.time(),
            datetime.now().isoformat(),
            self.current_status,
            f'{self.current_x:.4f}',
            f'{self.current_y:.4f}',
            f'{self.current_yaw:.4f}',
            int(self.has_icp_pose),
        ])
        self.has_icp_pose = False

    # ---- 统计 ----

    def print_summary(self):
        if not self.events:
            self.get_logger().warn('没有记录到任何恢复事件')
            return

        total = len(self.events)
        self_recover = [e for e in self.events if e['method'] == '主节点自恢复']
        icp_assist = [e for e in self.events if e['method'] == 'icp辅助']

        self.get_logger().info('=' * 50)
        self.get_logger().info('重定位实验统计')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'总恢复次数: {total}')
        self.get_logger().info(f'  主节点自恢复: {len(self_recover)}')
        self.get_logger().info(f'  副节点辅助:   {len(icp_assist)}')

        if self_recover:
            times = [e['elapsed_sec'] for e in self_recover]
            errs = [e['pos_error'] for e in self_recover]
            self.get_logger().info(f'自恢复 - 耗时: {min(times):.2f}~{max(times):.2f}s '
                                   f'(均值{sum(times)/len(times):.2f}s)')
            self.get_logger().info(f'自恢复 - 精度: {min(errs):.3f}~{max(errs):.3f}m '
                                   f'(均值{sum(errs)/len(errs):.3f}m)')

        if icp_assist:
            times = [e['elapsed_sec'] for e in icp_assist]
            errs = [e['pos_error'] for e in icp_assist]
            self.get_logger().info(f'ICP辅助 - 耗时: {min(times):.2f}~{max(times):.2f}s '
                                   f'(均值{sum(times)/len(times):.2f}s)')
            self.get_logger().info(f'ICP辅助 - 精度: {min(errs):.3f}~{max(errs):.3f}m '
                                   f'(均值{sum(errs)/len(errs):.3f}m)')

        self.get_logger().info('=' * 50)

    def destroy_node(self):
        self.print_summary()
        self.csv_file.close()
        self.get_logger().info(f'数据已保存至: {self.output_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # 解析参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', '-o', default=None, help='输出CSV文件路径')
    parsed, _ = parser.parse_known_args()

    if parsed.output:
        output_path = parsed.output
    else:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_path = f'reloc_experiment_{ts}.csv'

    node = RelocRecorder(output_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
