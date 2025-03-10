#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
from transforms3d.euler import euler2quat  # 使用 transforms3d 的 euler2quat
from tf2_ros import StaticTransformBroadcaster

class FormationUpdate(Node):
    def __init__(self):
        super().__init__('formation_update')

        # 获取参数
        self.declare_parameter('master_name', 'robot_master')
        self.declare_parameter('multi_mode', 'row')

        self.master_name = self.get_parameter('master_name').get_parameter_value().string_value
        self.multi_mode = self.get_parameter('multi_mode').get_parameter_value().string_value

        # 初始化位置
        self.x2, self.x3, self.y2, self.y3 = 0.0, 0.0, 0.0, 0.0
        self.set_initial_position()

        # 创建服务
        self.srv_row = self.create_service(Trigger, 'set_row', self.set_row_srv)
        self.srv_column = self.create_service(Trigger, 'set_column', self.set_column_srv)
        self.srv_triangle = self.create_service(Trigger, 'set_triangle', self.set_triangle_srv)

        # 创建静态坐标广播器
        self.broadcaster = StaticTransformBroadcaster(self)

        # 定时器频率（30 Hz）
        self.timer = self.create_timer(1.0 / 30.0, self.broadcast_transforms)

    def set_initial_position(self):
        if self.multi_mode == 'row':
            self.set_row_srv(None)
        elif self.multi_mode == 'column':
            self.set_column_srv(None)
        else:
            self.set_triangle_srv(None)

    def set_row_srv(self, request):
        self.x2, self.x3, self.y2, self.y3 = 0.0, 0.0, -0.7, 0.7
        return Trigger.Response(success=True)

    def set_column_srv(self, request):
        self.x2, self.x3, self.y2, self.y3 = -0.6, -1.2, 0.0, 0.0
        return Trigger.Response(success=True)

    def set_triangle_srv(self, request):
        self.x2, self.x3, self.y2, self.y3 = -0.7, -0.7, -0.4, 0.4
        return Trigger.Response(success=True)

    def broadcast_transforms(self):
        # 创建并组织被广播的消息 for robot_1
        robot_1 = TransformStamped()
        robot_1.header.stamp = self.get_clock().now().to_msg()
        robot_1.header.frame_id = f'{self.master_name}/base_footprint'
        robot_1.child_frame_id = 'point2'
        robot_1.transform.translation.x = float(self.x2)
        robot_1.transform.translation.y = float(self.y2)
        robot_1.transform.translation.z = 0.0

        qtn = euler2quat(0, 0, 0)  # 使用 transforms3d 的 euler2quat
        robot_1.transform.rotation.x = qtn[1]  # 注意 transforms3d 返回 (w, x, y, z)
        robot_1.transform.rotation.y = qtn[2]
        robot_1.transform.rotation.z = qtn[3]
        robot_1.transform.rotation.w = qtn[0]

        # 创建并组织被广播的消息 for robot_3
        robot_3 = TransformStamped()
        robot_3.header.stamp = self.get_clock().now().to_msg()
        robot_3.header.frame_id = f'{self.master_name}/base_footprint'
        robot_3.child_frame_id = 'point3'
        robot_3.transform.translation.x = float(self.x3)
        robot_3.transform.translation.y = float(self.y3)
        robot_3.transform.translation.z = 0.0

        robot_3.transform.rotation.x = qtn[1]
        robot_3.transform.rotation.y = qtn[2]
        robot_3.transform.rotation.z = qtn[3]
        robot_3.transform.rotation.w = qtn[0]

        # 广播器发送消息
        self.broadcaster.sendTransform(robot_1)
        self.broadcaster.sendTransform(robot_3)


def main(args=None):
    rclpy.init(args=args)
    node = FormationUpdate()

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
