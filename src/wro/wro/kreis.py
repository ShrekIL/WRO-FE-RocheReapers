#!/usr/bin/env python3
# encoding: utf-8
# ????(color tracking)
import os
import os.path
import cv2 as cv
import math
import queue
import rclpy
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi
import time
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, Trigger
from interfaces.srv import SetPoint, SetFloat64
from ros_robot_controller_msgs.msg import MotorsState, SetPWMServoState, PWMServoState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor

from .trajectory import Trajectory
from .icp import icp
from .utils import *
from .block_vison_camera import *
from .config import config
from .obstacle import Obstacle
from .lidar import lidar_to_obstacles
from .path_planing import generate_trajectories


class OjbectTrackingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.set_callback = False
        self.color_picker = None
        self.tracker = None
        self.is_running = False
        self.threshold = 0.1
        self.dist_threshold = 0.3
        self.lock = threading.RLock()
        self.image_sub = None
        self.result_image = None
        self.image_height = None
        self.image_width = None
        self.bridge = CvBridge()

        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

        self.set_speed(1)
        self.steer(-1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'stopped')
        
    def log(self, msg):
        self.get_logger().info('\033[1;32m%s\033[0m' % msg)

    def get_node_state(self, request, response):
        response.success = True
        return response
    
    def publish_servo_state(self, positions):
        servo_state = PWMServoState()
        servo_state.id = [3]
        servo_state.position = positions
        data = SetPWMServoState()
        data.state = [servo_state]
        data.duration = 0.02
        self.get_logger().info(f'Publishing servo state: {data}')
        self.servo_state_pub.publish(data)


    def steer(self, direction):
        """
        direction: 1 = left, -1 = right
        """
        direction = max(-1, min(1, direction))

        max_steer = 600
        norm_dir = 1500
        
        new_angle = norm_dir + direction * max_steer
        
        self.publish_servo_state([int(new_angle)])
        self.get_logger().info(f'Steering to {new_angle}')

    def set_speed(self, speed):
        """
        speed: -1 to 1
        """
        speed = - speed
        speed = max(-1, min(1, speed))
        
        t = Twist()
        t.linear.x = float(speed)
        self.mecanum_pub.publish(t)
        
def main():
    node = OjbectTrackingNode('object_tracking')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()