#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/08

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

class SetOdomNode(Node):
    def __init__(self):
        super().__init__('set_odom')
        
        self.robot_name = self.declare_parameter('robot_name', 'robot_1').value
        self.slam_methods = self.declare_parameter('slam_methods', 'gmapping').value
        self.x = self.declare_parameter('x', 0.0).value
        self.y = self.declare_parameter('y', 0.0).value

        self.odom_pub = self.create_publisher(Pose2D, f'{self.robot_name}/set_odom', 10)
        
        self.subscription = self.create_subscription(Odometry, f'{self.robot_name}/odom', self.odom_callback, 10)
        
        self.start_slam()
        
    def odom_callback(self, msg):
        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        
        self.get_logger().info(f'Setting odom for {self.robot_name}: x={self.x}, y={self.y}')
        self.odom_pub.publish(ppose)

        self.subscription.destroy()

    def start_slam(self):
        self.get_logger().info(f'Starting SLAM method: {self.slam_methods}')
        os.system(f'ros2 launch slam slam_base.launch.py slam_methods:={self.slam_methods}')

def main(args=None):
    rclpy.init(args=args)
    set_odom_node = SetOdomNode()
    rclpy.spin(set_odom_node)
    set_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

