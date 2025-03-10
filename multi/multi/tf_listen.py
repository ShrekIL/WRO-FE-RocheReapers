#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from transforms3d.euler import quat2euler  # 使用 transforms3d 替代 tf_transformations
from sdk import pid

def qua2rpy(x, y, z, w):
    roll, pitch, yaw = quat2euler([w, x, y, z])
    return roll, pitch, yaw

class SlaveTFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')

        # 获取参数
        self.declare_parameter('cmd_vel', '/robot_1/hiwonder_controller/cmd_vel')
        self.declare_parameter('base_frame', 'robot_1/base_footprint')  # 不要以 '/' 开头
        self.declare_parameter('target_frame', 'row2')  # 不要以 '/' 开头

        self.cmd_vel = self.get_parameter('cmd_vel').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        # 创建发布器
        self.robot_vel_publisher = self.create_publisher(Twist, self.cmd_vel, 10)

        # TF2 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 创建 PID 控制器
        self.pid_x = pid.PID(1.5, 0, 0)
        self.pid_y = pid.PID(1.5, 0, 0)
        self.pid_z = pid.PID(0.1, 0, 0)

        # 创建定时器，每隔一段时间执行 update 函数
        self.timer = self.create_timer(1.0 / 30.0, self.update)

    def update(self):
        msg = Twist()
        try:
            # 查找相对的 tf
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.base_frame, self.target_frame, now)
        except Exception as e:
            self.get_logger().warn(f"Failed to get transform: {e}")
            self.robot_vel_publisher.publish(msg)
            return

        # 提取平移和旋转
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
        if abs(angle) < 4:
            angle = 0
        self.pid_z.update(angle)
        angular_z = -self.pid_z.output

        # 限制速度
        linear_x = max(min(linear_x, 0.4), -0.4)
        linear_y = max(min(linear_y, 0.4), -0.4)
        angular_z = max(min(angular_z, 0.8), -0.8)

        # 设置消息并发布
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z

        self.robot_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SlaveTFListener()

    try:
        # 自定义循环来替代 spin
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
