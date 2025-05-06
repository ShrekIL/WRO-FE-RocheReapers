#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from transforms3d.euler import quat2euler
from sdk import pid


def qua2rpy(x, y, z, w):
    # 使用 transforms3d 的 quat2euler 获取 roll, pitch, yaw
    roll, pitch, yaw = quat2euler([w, x, y, z])
    return roll, pitch, yaw


class SlaveTFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        
        # 获取参数
        self.declare_parameter('cmd_vel', '/robot_1/robot_controller/cmd_vel')
        self.declare_parameter('base_frame', 'robot_1/base_footprint')  # 移除前导斜杠
        self.declare_parameter('target_frame', 'point2')  # 移除前导斜杠

        cmd_vel = self.get_parameter('cmd_vel').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value.strip('/')  # 去掉斜杠
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value.strip('/')  # 去掉斜杠

        # 发布器
        self.robot_vel_publisher = self.create_publisher(Twist, cmd_vel, 10)

        # TF2 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # PID 控制器
        self.pid_x = pid.PID(1.5, 0, 0)
        self.pid_y = pid.PID(1.5, 0, 0)
        self.pid_z = pid.PID(0.1, 0, 0)

        # 定时器频率
        self.update_frequency = 30.0  # Hz

    def update(self):
        msg = Twist()
        try:
            # 获取相对的tf
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.base_frame, self.target_frame, now)
        except Exception as e:
            self.get_logger().warn(f"Failed to get transform: {e}")
            self.robot_vel_publisher.publish(msg)
            return

        # 提取位置信息
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        rot = trans.transform.rotation
        _, _, angle = qua2rpy(rot.x, rot.y, rot.z, rot.w)

        # PID 控制
        self.pid_x.SetPoint = 0
        if abs(x) < 0.03:
            x = 0
        self.pid_x.update(x)
        linear_x = -self.pid_x.output

        self.pid_y.SetPoint = 0
        if abs(y) < 0.03:
            y = 0
        self.pid_y.update(y)
        linear_y = -self.pid_y.output

        self.pid_z.SetPoint = 0
        if abs(angle) < 5:
            angle = 0
        self.pid_z.update(angle)
        angular_z = -self.pid_z.output

        # 限制速度
        linear_x = max(min(linear_x, 0.25), -0.25)
        linear_y = max(min(linear_y, 0.25), -0.25)
        angular_z = max(min(angular_z, 0.4), -0.4)

        # 设置消息并发布
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z

        self.robot_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlaveTFListener()

    try:
        # 自定义循环，替代 rclpy.spin()
        while rclpy.ok():
            # 调用 update 方法
            node.update()
            # 处理一次回调，非阻塞
            rclpy.spin_once(node, timeout_sec=1.0 / node.update_frequency)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
