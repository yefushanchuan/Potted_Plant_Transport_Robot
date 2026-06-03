#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
import sys
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from robot_base.action import OperatePot


def enter_slave_mode():
    """发送 action_type=3 (Slave) 切换到从机模式。"""
    node = rclpy.create_node('_enter_slave')
    client = ActionClient(node, OperatePot, 'operate_pot')
    if not client.wait_for_server(timeout_sec=3.0):
        node.get_logger().error('operate_pot action server 不可用')
        node.destroy_node()
        return False
    goal = OperatePot.Goal()
    goal.action_type = 3
    goal.rack_index = 0
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
    ok = future.result() is not None
    node.destroy_node()
    return ok


class angleCheckNodeV2(Node):
    def __init__(self, target_angle):
        super().__init__('angle_check_node')

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imutopic, imu_qos)

        self.target_angle = target_angle  # 要旋转的总角度 (rad)
        self.r_speed     = 0.3            # rad/s
        self.r_tolerance = 0.05           # rad
        self.start_test = True

        self.cmd_vel = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 1)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.cntFre = 0

        # IMU wz 积分
        self.prev_wz = None
        self.prev_time = None
        self.accumulated = 0.0
        self.imu_yaw = 0.0  # IMU 四元数算出的 yaw（仅显示用）
        self.last_cb_time = None   # 最近一次回调时间
        self.cb_count = 0          # 回调计数
        self.last_accumulated = 0.0  # 上次打印时的累计值

    def imutopic(self, data):
        # 用 IMU 的 wz 积分（不受 ±π 限制）
        now = self.get_clock().now().nanoseconds / 1e9
        wz = data.angular_velocity.z  # yaw 角速度 rad/s

        self.cb_count += 1
        self.last_cb_time = now

        if self.prev_time is not None:
            dt = now - self.prev_time
            if 0 < dt < 0.1:  # 合理的 dt 范围
                self.accumulated += wz * dt

        self.prev_wz = wz
        self.prev_time = now

        # 同时记录 IMU 四元数 yaw（仅显示）
        x, y, z, w = data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
        self.imu_yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def timer_callback(self):
        if self.prev_time is None:
            return  # 还没收到 IMU 数据

        if self.cntFre == 20:
            self.cntFre = 0
            delta = self.accumulated - self.last_accumulated
            now = self.get_clock().now().nanoseconds / 1e9
            cb_age = (now - self.last_cb_time) if self.last_cb_time else -1
            self.get_logger().info(
                'imu_yaw=%.3f  累计=%.3f  wz=%.4f  delta=%.4f  cb_count=%d  cb_age=%.1fs' %
                (self.imu_yaw, self.accumulated, self.prev_wz or 0, delta, self.cb_count, cb_age))
            if abs(delta) < 0.001:
                self.get_logger().warn('⚠️ 累计角度停滞! wz=%.6f  IMU回调仍在=%s' %
                                       (self.prev_wz or 0, '是' if cb_age < 1.0 else '否(超时)'))
            self.last_accumulated = self.accumulated
        self.cntFre += 1

        remaining = self.target_angle - self.accumulated
        if abs(remaining) < self.r_tolerance or not self.start_test:
            self.cmd_vel.publish(Twist())
            self.get_logger().info('Target reached! Stopping robot.')
            self.get_logger().info('accumulated: %.3f rad (%.1f°)' %
                                   (self.accumulated, math.degrees(self.accumulated)))
            sys.exit(0)
        else:
            move_cmd = Twist()
            move_cmd.angular.z = self.r_speed if remaining > 0 else -self.r_speed
            self.cmd_vel.publish(move_cmd)


def main(args=None):
    rclpy.init(args=args)

    # 默认转一圈 (2π)
    target_angle = 2.0 * math.pi
    if len(sys.argv) >= 2:
        target_angle = float(sys.argv[1])

    if enter_slave_mode():
        print('[angular_calibration] 已切换到 Slave 模式')
    else:
        print('[angular_calibration] ⚠️ 切换 Slave 模式失败')

    print(f'[angular_calibration] 目标旋转: {target_angle:.2f} rad ({math.degrees(target_angle):.1f}°)')

    node = angleCheckNodeV2(target_angle)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
