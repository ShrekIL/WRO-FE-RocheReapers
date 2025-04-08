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
from .vision import *
from .config import config
from .obstacle import Obstacle


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

        #self.image_sub = self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 1)

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, qos)

        self.last_laser_scan = None
        
        self.pos_lock = threading.Lock()
        self.rel_to_odom_pos = None
        self.rel_to_odom_rot = None
        
        self.position = [0,0]
        self.position_history = []

        plt.figure(figsize=(6,10))

        # m/s
        self.speed = 0
        self.dx = 0 #todo
        self.dy = 0 #todo
        
        self.rotation = 0
        
        # turn rad/s
        self.omega = 0
        
        self.goal = np.array([0,1])
        
        #todo
        self.dt = 0

        
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
        
    def calculate_dynamic_window(self):
        """
        Calculates the dynamic window
        Returns:
            [v_min, v_max, yaw_rate_min, yaw_rate_max]
        """
        # Dynamic window from robot specification
        Vs = [config.min_speed, config.max_speed,
            -config.max_yaw_rate, config.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [self.speed - config.max_accel * self.dt,
            self.speed + config.max_accel * self.dt,
            self.omega - config.max_delta_yaw_rate * self.dt,
            self.omega + config.max_delta_yaw_rate * self.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        
        return dw
    
    def obstace_cost(self, trajectory: Trajectory, obstacle: Obstacle):
        dx = 0
        dy = 0
        
        dist = obstacle.get_distance(trajectory)
        return dist

    def goal_cost(self, trajectory):
        dx = 0
        dy = 0
        
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

        
    def predict_trajectory(self, v, yaw) -> Trajectory:
        """
        Predict a trajectory based on input
        """
        tr = Trajectory()
        tr.pos_init = self.position

        dx = 0
        dy = 0
        
        tr.pos = [self.position + dx, self.position + dy]


    def dwa_control(self):
        pass

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        self.image_height, self.image_width = rgb_image.shape[:2]
        self.get_logger().info('\033[1;32m%s\033[0m' % 'image callback')
        rgb_image = cv.cvtColor(rgb_image, cv.COLOR_BGR2RGB)
        
        res = detect_block(rgb_image)
        color, (x, y) = res
        self.log(f"color: {color} x: {x} y: {y}")     
        
        direction = x / 640 - 0.5
        
        self.set_speed(1)
        
        self.steer(-1)
        
        cv.imshow("test", rgb_image)

    def lidar_callback(self, lidar_data):
        #self.get_logger().info('\033[1;32m%s\033[0m' % 'lidar callback')

        angles = np.linspace(lidar_data.angle_min, lidar_data.angle_max, len(lidar_data.ranges))
        distances = np.array(lidar_data.ranges)
        distances = np.where(np.isnan(distances), 0, distances)
        distances = np.where(np.isinf(distances), 0, distances)
        distances = np.where(distances > 3, 0, distances)
        distances = np.where(distances < 0.1, 0, distances)
        
        mask = distances != 0
        distances = distances[mask]
        angles = angles[mask]
                
        oy = -np.cos(angles) * distances
        ox = np.sin(angles) * distances
        scan = np.column_stack((ox, oy))
        
        if os.path.exists("/home/kuttelr/liadar.npy"):
            return
        np.save("/home/kuttelr/liadar.npy", scan)
        
        # Step 1: Cluster points into separate lines using DBSCAN
        dbscan = DBSCAN(eps=0.2, min_samples=2)
        labels = dbscan.fit_predict(scan)

        unique_labels = set(labels)

        # Step 2: Fit RANSAC for each cluster
        plt.cla()
        
        lines = []

        for label in unique_labels:
            if label == -1:  # Ignore noise points
                continue
            
            # Get points belonging to the current cluster
            cluster_points = scan[labels == label]
            X_cluster = cluster_points[:, 0].reshape(-1, 1)
            Y_cluster = cluster_points[:, 1]

            # Fit RANSAC to the cluster
            ransac = RANSACRegressor()
            ransac.fit(X_cluster, Y_cluster)
            
            # Get the fitted line parameters
            m = ransac.estimator_.coef_[0]
            b = ransac.estimator_.intercept_

            # Generate fitted line points
            X_fit = np.linspace(min(X_cluster), max(X_cluster), 100).reshape(-1, 1)
            Y_fit = ransac.predict(X_fit)

            lines.append([X_fit, Y_fit])

            # Plot cluster points and fitted line
            plt.scatter(X_cluster, Y_cluster, label=f"Cluster {label}")
            plt.plot(X_fit, Y_fit, linewidth=2)

        plt.scatter([0], [0], label="Robot", c="red")
        plt.legend()
        plt.xlabel("X")
        plt.xlim((-3,3))
        plt.ylim((-3,3))
        plt.ylabel("Y")
        plt.title("RANSAC Line Fitting for Multiple LiDAR Clusters")
        
        
        
        plt.pause(0.0001)

        self.log(f" {lines}")

        self.last_laser_scan = scan
        
def main():
    node = OjbectTrackingNode('object_tracking')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
