#!/usr/bin/env python3
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from tracemalloc import start
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from geometry_msgs.msg import Twist
import sys
import tf_transformations
from tf2_ros import LookupException, ConnectivityException, TransformException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from time import time
import math



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('distance_check_node')

        
        self.declare_parameter('time', 0.02)
        self.periodtime = self.get_parameter(
            'time').get_parameter_value().double_value
        self.get_logger().info(f"Timer period set to: {self.periodtime} seconds, {1.0/self.periodtime:.2f} Hz")

        try:
            if len(sys.argv) == 2: # 传入1个参数则默认是前进距离x
                self.test_dx = float(sys.argv[1])  
                self.test_dy = 0.0
            elif len(sys.argv) == 3: # 传入2个参数则分别对应x、y方向位移
                self.test_dx = float(sys.argv[1])
                self.test_dy = float(sys.argv[2])
            else:
                self.test_dx = 1.0 # meters
                self.test_dy = 0.0 # meters
        except Exception: # 读取参数失败则默认前进一米
            self.test_dx = 1.0 # meters
            self.test_dy = 0.0 # meters

        self.speed       =  0.10 # meters per second
        self.speed       =  0.0 # meters per second
        self.speed_yaw   =  45.0  # radians per second
        self.speed_yaw   =  math.radians(self.speed_yaw) # 转换为弧度制
        self.test_dtime  =  math.radians(360 * 1.0) / self.speed_yaw  # seconds needed to rotate 360 degrees at speed_yaw
        self.tolerance   = 0.01 # meters
        self.start_test = True
            
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("Bring up rqt_reconfigure to control the test.")

        # ros2_control 的 diff_drive_controller 在 use_stamped_vel=false 时订阅的是
        # /diff_drive_controller/cmd_vel_unstamped (geometry_msgs/msg/Twist)。
        self.cmd_vel = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped',  1)
      
        self.timer = self.create_timer(self.periodtime, self.timer_callback) # 单位是秒


        self.from_frame_rel = 'base_link'
        self.to_frame_rel = 'odom'

        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.start_time = None
        self.dis_count = 0
        # self.pub_once()



        


    def timer_callback(self):
        move_cmd = Twist()
        
        now = rclpy.time.Time()
        trans = self.tf_buffer.lookup_transform(
            self.to_frame_rel,
            self.from_frame_rel,
            now)
        if self.start_x is None or self.start_y is None or self.start_yaw is None:
            self.start_time = time()
            self.start_x = trans.transform.translation.x
            self.start_y = trans.transform.translation.y
            self.start_yaw = tf_transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])[2]
            self.get_logger().info(f"Initialized start position: x={self.start_x}, y={self.start_y}, yaw={self.start_yaw}")
            # self.target_x = self.x + math.cos(self.yaw) * self.test_dx - math.sin(self.yaw) * self.test_dy
            # self.target_y = self.y + math.sin(self.yaw) * self.test_dx + math.cos(self.yaw) * self.test_dy
        self.x = trans.transform.translation.x
        self.y = trans.transform.translation.y
        self.yaw = tf_transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])[2]
        self.get_logger().info(f'\nself.x：{self.x:.4f}, self.y：{self.y:.4f}, self.yaw：{self.yaw:.4f}, time elapsed: {time() - self.start_time:.4f} seconds, target time: {self.test_dtime:.4f} seconds')
        
        # Re-implementing the logic cleanly:
        current_distance = math.sqrt(pow((self.x - self.start_x), 2) + pow((self.y - self.start_y), 2))
        self.get_logger().info(f'Distance Traveled: {current_distance:.4f} / {self.test_dx:.4f}')
        current_yaw_change = math.degrees(abs(self.yaw - self.start_yaw))
        self.get_logger().info(f'Yaw Change: {current_yaw_change:.4f} degrees')

        if current_distance >= self.test_dx or time() - self.start_time >= self.test_dtime:
            self.get_logger().info('Target distance reached! Stopping robot.')
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.cmd_vel.publish(move_cmd)
            self.dis_count += 1
            if self.dis_count >= 100:
                sys.exit(0)
        else:
            move_cmd.linear.x = self.speed
            move_cmd.angular.z = self.speed_yaw
            self.cmd_vel.publish(move_cmd)

    def get_pose(self):
        # Get the current transform between the odom and base frames
        try:
            now = rclpy.time.Time()
            (trans, rot)  = self.tf_buffer.lookup_transform(
            self.to_frame_rel,
            self.from_frame_rel,
            now)
        except (Exception, ConnectivityException, LookupException):
            self.get_logger().info("TF Exception")
            return

        return (trans, rot)

    def pub_once(self):
        self.get_logger().info("Starting movement...")
        move_cmd = Twist()
        move_cmd.linear.x = 0.4
        move_cmd.angular.z = 0.0
        time_start = time()
        now = time()
        while now - time_start < 2:
            self.cmd_vel.publish(move_cmd)
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.cmd_vel.publish(move_cmd)
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
