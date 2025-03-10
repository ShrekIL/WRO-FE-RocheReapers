#!/usr/bin/env python3
# encoding: utf-8
# @aiden
# @2022/07/28
import cv2
import rclpy
from rclpy.node import Node
import tf2_ros
from math import degrees, atan2, asin
import numpy as np
from std_msgs.msg import Header, Int32
from geometry_msgs.msg import PolygonStamped, Point32

rect_width_half = 150
rect_height_half = 130

def qua2rpy(x, y, z, w):
    roll = degrees(atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = degrees(asin(2 * (w * y - x * z)))
    yaw = degrees(atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y)))
    return roll, pitch, yaw

def rotate(ps, m):
    pts = np.float32(ps).reshape([-1, 2])  # Points to transform
    pts = np.hstack([pts, np.ones([len(pts), 1])]).T
    target_point = np.dot(m, pts).astype(np.int32)
    target_point = [[target_point[0][x], target_point[1][x]] for x in range(len(target_point[0]))]
    return target_point

def rotate_point(center_point, corners, angle):
    M = cv2.getRotationMatrix2D((center_point[0], center_point[1]), angle, 1)
    out_points = rotate(corners, M)
    return out_points

class VirtualWallNode(Node):
    def __init__(self):
        super().__init__('virtual_wall')
        self.declare_parameter('base_frame', 'robot_1/base_footprint')
        self.declare_parameter('map_frame', 'robot_1/map')

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        self.delete_publish = self.create_publisher(Int32, 'delete_wall', 1)
        self.add_publish = self.create_publisher(PolygonStamped, 'add_wall', 1)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0 / 30, self.timer_callback)

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        x = int(trans.transform.translation.x * 1000)
        y = int(trans.transform.translation.y * 1000)
        rpy = qua2rpy(trans.transform.rotation.x, trans.transform.rotation.y, 
                      trans.transform.rotation.z, trans.transform.rotation.w)

        left_up = [x - rect_width_half, y - rect_height_half]
        right_up = [x + rect_width_half, y - rect_height_half]
        left_down = [x - rect_width_half, y + rect_height_half]
        right_down = [x + rect_width_half, y + rect_height_half]

        out_points = rotate_point([x, y], [left_up, right_up, right_down, left_down], -rpy[-1])

        pose = PolygonStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        points = []

        for i in out_points:
            point = Point32()
            point.x = i[0] / 1000.0
            point.y = i[1] / 1000.0
            point.z = 0
            points.append(point)

        pose.polygon.points = points
        self.add_publish.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = VirtualWallNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
