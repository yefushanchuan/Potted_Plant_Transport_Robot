#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import math
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from time import time
from rclpy.action import ActionClient
from robot_base.action import OperatePot


def enter_slave_mode():
    """发送 action_type=3 (Slave) 切换到从机模式，使底盘响应速度命令。"""
    node = rclpy.create_node('_enter_slave')
    client = ActionClient(node, OperatePot, 'operate_pot')
    if not client.wait_for_server(timeout_sec=3.0):
        node.get_logger().error('operate_pot action server 不可用')
        node.destroy_node()
        return False
    goal = OperatePot.Goal()
    goal.action_type = 3  # Slave
    goal.rack_index = 0
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
    ok = future.result() is not None
    node.destroy_node()
    return ok


class LinearCalibrationNode(Node):
    def __init__(self, target_distance, speed):
        super().__init__('linear_calibration_node')

        self.target_distance = target_distance
        self.speed = speed

        self.cmd_vel = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_x = None
        self.start_y = None
        self.start_time = None

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(
            f"Linear calibration: target={self.target_distance}m, speed={self.speed}m/s")

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
        except Exception:
            self.get_logger().warn('TF lookup failed, waiting...')
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.start_time = time()
            self.get_logger().info(
                f"Start position: x={x:.4f}, y={y:.4f}")

        traveled = math.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)

        if traveled >= self.target_distance:
            elapsed = time() - self.start_time
            actual_speed = self.target_distance / elapsed if elapsed > 0 else 0
            self.get_logger().info('--- Calibration Complete ---')
            self.get_logger().info(
                f'Target distance:  {self.target_distance:.4f} m')
            self.get_logger().info(
                f'Actual distance:  {traveled:.4f} m')
            self.get_logger().info(
                f'Error:            {traveled - self.target_distance:.4f} m')
            self.get_logger().info(
                f'Time elapsed:     {elapsed:.2f} s')
            self.get_logger().info(
                f'Average speed:    {actual_speed:.4f} m/s')
            self.get_logger().info(
                f'Lateral drift:    {abs(y - self.start_y):.4f} m')
            self.cmd_vel.publish(Twist())
            sys.exit(0)
        else:
            move_cmd = Twist()
            move_cmd.linear.x = self.speed
            self.cmd_vel.publish(move_cmd)
            self.get_logger().info(
                f'Traveled: {traveled:.3f} / {self.target_distance:.3f} m',
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)

    if enter_slave_mode():
        print('[linear_calibration] 已切换到 Slave 模式')
    else:
        print('[linear_calibration] ⚠️ 切换 Slave 模式失败')

    target_distance = 1.0
    speed = 0.1

    if len(sys.argv) >= 2:
        target_distance = float(sys.argv[1])
    if len(sys.argv) >= 3:
        speed = float(sys.argv[2])

    node = LinearCalibrationNode(target_distance, speed)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
