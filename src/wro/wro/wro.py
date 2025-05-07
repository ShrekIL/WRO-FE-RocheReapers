#!/usr/bin/env python3
# encoding: utf-8
import os
import os.path
import cv2 as cv
import math
import queue
import rclpy
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi, atan2, tan # Added atan2, tan
import time
import threading
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, Trigger
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState # Simplified imports
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from .lidar import LidarResult
from .control import Control




class OjbectTrackingNode(Node):
    def __init__(self, name):
        self.__node_name = name
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.get_logger().info(f"Initializing node '{self.__node_name}'...")

        self.set_callback = False
        self.is_running = False

        self.lock = threading.RLock() # General lock
        self.image_sub = None
        self.result_image = None
        self.image_height = None
        self.image_width = None
        self.bridge = CvBridge()

        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)

        lidar_qos = QoSProfile(
             depth=1,
             reliability=QoSReliabilityPolicy.BEST_EFFORT, # Or RELIABLE
             durability=QoSDurabilityPolicy.VOLATILE)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, lidar_qos)

#        self.position = np.array([0.5, 0.5]) # Initial estimated position (e.g., near corner)
#        self.rotation = np.radians(45.0) # Initial estimated orientation
        self.pose_lock = threading.Lock() # Lock specifically for pose variables
        self.position_history = [] # Keep if needed for plotting paths

        # Last commanded/intended controls (used for MCL prediction)
        self.current_velocity = 0.0
        self.current_steering_angle = 0.0 # In radians


        # --- Visualization Setup ---
        plt.figure() # Create figure for animation
        plt.ion() # Turn on interactive mode
        plt.show() # Show the plot window

        self.get_logger().info('\033[1;32mNode initialized successfully.\033[0m')

        self.control = Control(self.get_logger())        
        
        # delta time in seconds
        self.dt = 0
        self.prev_time = time.time()
        
#        self.rot_drift = 0


    def log(self, msg):
        self.get_logger().info('\033[1;32m%s\033[0m' % msg)

    def publish_servo_state(self, positions):
        # Ensure positions are integers
        int_positions = [int(p) for p in positions]
        servo_state = PWMServoState()
        servo_state.id = [3] # servo ID 3 for steering
        servo_state.position = int_positions # Use integer positions
        data = SetPWMServoState()
        data.state = [servo_state]
        data.duration = 0.02 # Use float for duration
        # self.get_logger().info(f'Publishing servo state: {data}') # Can be verbose
        self.servo_state_pub.publish(data)

    def steer(self, direction):
        """
        Sets the steering command based on direction (-1 to 1).
        """
        direction = max(-1.0, min(1.0, float(direction))) # Ensure float and clamp

        self.current_steering_angle = direction

        max_steer_offset = 600 # Servo range offset
        norm_dir_servo = 1500 # Servo center value
        new_angle_servo = norm_dir_servo + direction * max_steer_offset
        self.publish_servo_state([int(new_angle_servo)]) # Send command to servo

    def set_speed(self, speed):
        """
        Sets the linear velocity command.
        """
        with self.lock: # Use general lock or a specific control lock
            # Assuming speed argument IS the desired velocity in m/s
            # Clamp if necessary based on robot limits
            self.current_velocity = speed
            # self.get_logger().info(f"Commanded velocity: {self.current_velocity:.2f} m/s")

            # --- Publish Twist Command ---
            t = Twist()
            # Use the *intended* velocity for the command, even if MCL uses a slightly different internal value
            # The sign convention might need adjustment based on your controller setup
            t.linear.x = float(speed) # Send original requested speed value
            self.mecanum_pub.publish(t)

    def image_callback(self, ros_image):
        pass

    


    def plot_lidar(self, lidar_data):
        angles = np.linspace(lidar_data.angle_min, lidar_data.angle_max, len(lidar_data.ranges))
        distances = np.array(lidar_data.ranges)
        distances = np.where(np.isnan(distances), 0, distances)
        distances = np.where(np.isinf(distances), 0, distances)
        distances = np.where(distances > 3, 0, distances)
        distances = np.where(distances < 0.1, 0, distances)
        
        mask = distances != 0
        distances = distances[mask]
        angles = angles[mask]
        
        angles = angles + np.pi
                
        oy = np.cos(angles) * distances
        ox = -np.sin(angles) * distances
        scan = np.column_stack((ox, oy))
        
        plt.cla()
        plt.grid(True)
        plt.xlim((3, -3)) # rescale y axis, to match the grid orientation
        plt.ylim((-3, 3)) # rescale y axis, to match the grid orientation
        plt.scatter(ox, oy, s=2, c="blue") # lines from 0,0 to the
        plt.xlabel("X")
        plt.ylabel("Y")

        plt.pause(0.0001)

 
        self.last_laser_scan = scan
         

    def lidar_callback(self, lidar_data):
        self.lidar_result = LidarResult(lidar_data)
        
        self.dt = time.time() - self.prev_time
        self.prev_time = time.time()
        

        self.control.update_lidar(lidar_data)
        speed, stearing = self.control.get_control_strategy()
        
        self.steer(stearing)
        #self.set_speed(speed)
        
        return
        
        angles = np.linspace(lidar_data.angle_min, lidar_data.angle_max, len(lidar_data.ranges))
        distances = np.array(lidar_data.ranges)
        distances = np.where(np.isnan(distances), 0, distances)
        distances = np.where(np.isinf(distances), 0, distances)
        distances = np.where(distances > 3, 0, distances)
        distances = np.where(distances < 0.1, 0, distances)
        
        mask = distances != 0
        ranges = distances[mask]
        angles = angles[mask]
    
        rot_drift = np.clip(rot_drift, -20, 20)
    
        frame_count += 1
        current_time = frame_count * DT

        # --- Simple Example Control Logic ---
        # Drive forward, turn slightly left initially, then maybe turn right
        velocity_cmd = 0.3 # m/s
        steering_cmd = 0.0 # radians
        
        color, (obstacle_distance, obstacle_angle) = detect_block(None)

        if color is not None:
            print(f"Obstacle ({color}) distance: {obstacle_distance:.2f} m, angle: {math.degrees(obstacle_angle):.1f}°")


        distance_to_front = ranges[round(len(ranges) / 2)]
        distance_to_left = ranges[-round(len(ranges) / 8)]
        distance_to_right = ranges[round(len(ranges) / 8)]

        distance_to_left_l = ranges[-round(len(ranges) / 8) + 8]
        distance_to_left_r = ranges[-round(len(ranges) / 8) - 8]


        distance_to_front_l = ranges[round(len(ranges) / 2) + 8]
        distance_to_front_r = ranges[round(len(ranges) / 2) - 8]
        
        distance_to_right_l = ranges[round(len(ranges) / 8) + 8]
        distance_to_right_r = ranges[round(len(ranges) / 8) - 8]
        
        if state == "MOVING":
            if rot_drift > 1:
                steering_cmd = -1
                rot_drift -= 1
            elif rot_drift < -1:
                steering_cmd = 1
                rot_drift += 1
                    
            print(f"Moving dist left: {distance_to_left}")
            if obstacle_distance is not None and obstacle_distance < 0.5:
                print(f"CHANGING STATE: MOVING -> OBSTACLE_AVOIDANCE")
                rot_drift /= 3
                state = "OBSTACLE_AVOIDANCE"
                steering_cmd = 1
            
            elif distance_to_front < 0.8 and obstacle_distance is None and max(distance_to_left, distance_to_left_l, distance_to_left_r) > 0.8 and max(distance_to_right, distance_to_right_l, distance_to_right_r) < 1:
                print(f"CHANGING STATE: MOVING -> TURNING")
                state = "TURNING"
                steering_cmd = 1

        if state == "OBSTACLE_AVOIDANCE":
            print(f"OBSTACLE_AVOIDANCE: angle={obstacle_angle} distance={obstacle_distance} rot_drift={rot_drift}")
            if obstacle_distance is None:
                print(f"CHANGING STATE: OBSTACLE_AVOIDANCE -> MOVING")
                state = "MOVING"
                steering_cmd = 0
            elif obstacle_distance < 0.2 and not ((obstacle_angle > 0.7 and color == "red") or (obstacle_angle < -0.7 and color == "green")):
                print(f"CHANGING STATE: OBSTACLE_AVOIDANCE -> OBSTACLE_AVOIDANCE_PANIC")
                state = "OBSTACLE_AVOIDANCE_PANIC"
            else:
                if color == "red" and obstacle_angle < 0.7:
                    steering_cmd = -1
                    rot_drift -= 1.25
                elif color == "green" and obstacle_angle > -0.7:
                    steering_cmd = 1
                    rot_drift += 1.25

        if state == "OBSTACLE_AVOIDANCE_PANIC":
            print(f"OBSTACLE_AVOIDANCE_PANIC: angle={obstacle_angle} distance={obstacle_distance} rot_drift={rot_drift}")
            if obstacle_distance is None:
                print(f"CHANGING STATE: OBSTACLE_AVOIDANCE_PANIC -> MOVING")
                state = "MOVING"
            elif obstacle_distance > 0.5:
                print(f"CHANGING STATE: OBSTACLE_AVOIDANCE_PANIC -> OBSTACLE_AVOIDANCE")
                state = "OBSTACLE_AVOIDANCE"
            else:
                velocity_cmd = -0.3
                if obstacle_angle > 0.2:
                    steering_cmd = 0.2
                    rot_drift += 0.2
                elif obstacle_angle < -0.2:
                    steering_cmd = -0.2
                    rot_drift -= 0.2
                

        if state == "TURNING":
            print(f"TURNING: {prev_dist_front=} {distance_to_front=}")
            if abs(distance_to_front_r - distance_to_front_l) < 0.15:
                print(f"CHANGING STATE: TURNING -> MOVING")
                state = "MOVING"
                steering_cmd = 0
            if obstacle_distance is not None and obstacle_distance < 0.5:
                print(f"CHANGING STATE: TURNING -> OBSTACLE_AVOIDANCE")
                state = "OBSTACLE_AVOIDANCE"
                velocity_cmd = 0.3
            else:
                steering_cmd = 1


        print("Steering command:", math.degrees(steering_cmd))
        print("Velocity command:", velocity_cmd)

        prev_dist_front = distance_to_front

        self.steer(steering_cmd)
        self.set_speed(velocity_cmd)        
        

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OjbectTrackingNode('wro') # Renamed node
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C detected, shutting down.")
    except Exception as e:
        raise e
        if node:
             node.get_logger().fatal(f"Unhandled exception: {e}")
        else:
             print(f"Exception during node creation: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()