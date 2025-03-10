#!/usr/bin/env python3
# tf_follower.py
import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
import math

class TFFollower(Node):
    def __init__(self):
        super().__init__('tf_follower')

        # 参数配置
        self.declare_parameter('leader_frame', 'robot1/base_footprint')
        self.declare_parameter('follower_frame', 'robot2/base_footprint')
        self.declare_parameter('offset_x', -1.0)  # 在robot1的后方1米
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)

        self.leader_frame = self.get_parameter('leader_frame').get_parameter_value().string_value
        self.follower_frame = self.get_parameter('follower_frame').get_parameter_value().string_value
        self.offset_x = self.get_parameter('offset_x').get_parameter_value().double_value
        self.offset_y = self.get_parameter('offset_y').get_parameter_value().double_value
        self.offset_z = self.get_parameter('offset_z').get_parameter_value().double_value

        # TF2缓存和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TF2发布器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        try:
            # 获取leader的最新位姿（相对于世界坐标系）
            leader_transform = self.tf_buffer.lookup_transform(
                'odom',  # 目标帧
                self.leader_frame,  # 源帧
                rclpy.time.Time())  # 当前时间

            # 计算follower的目标位姿
            follower_transform = TransformStamped()

            follower_transform.header.stamp = self.get_clock().now().to_msg()
            follower_transform.header.frame_id = 'odom'
            follower_transform.child_frame_id = self.follower_frame

            # 将偏移应用到leader的位姿
            # 这里假设偏移是相对于leader的局部坐标系
            # 需要将偏移转换到世界坐标系

            # 先计算偏移在世界坐标系下的坐标
            quat = leader_transform.transform.rotation
            siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            # 计算偏移后的世界坐标
            dx_world = self.offset_x * math.cos(yaw) - self.offset_y * math.sin(yaw)
            dy_world = self.offset_x * math.sin(yaw) + self.offset_y * math.cos(yaw)
            dz_world = self.offset_z

            follower_transform.transform.translation.x = leader_transform.transform.translation.x + dx_world
            follower_transform.transform.translation.y = leader_transform.transform.translation.y + dy_world
            follower_transform.transform.translation.z = leader_transform.transform.translation.z + dz_world

            # 保持与leader相同的姿态
            follower_transform.transform.rotation = leader_transform.transform.rotation

            # 发布follower的位姿
            self.tf_broadcaster.sendTransform(follower_transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn('Leader transform not available yet.')

def main(args=None):
    rclpy.init(args=args)
    node = TFFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
