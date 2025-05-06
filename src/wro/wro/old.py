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
# import sdk.pid as pid # Assuming not directly needed for MCL integration
# import sdk.common as common # Assuming not directly needed
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, Trigger
# from interfaces.srv import SetPoint, SetFloat64 # Assuming not directly needed
# from ros_robot_controller_msgs.msg import MotorsState, SetPWMServoState, PWMServoState # Keep if needed
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState # Simplified imports
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy # Added DurabilityPolicy
from nav_msgs.msg import Odometry # Keep if used elsewhere
# from sklearn.cluster import DBSCAN # Keep if used elsewhere (e.g., lidar_to_obstacles)
# from sklearn.linear_model import RANSACRegressor # Keep if used elsewhere

# --- Assuming MCL code is in a separate file named 'mcl_module.py' ---
# If in the same file, remove the 'mcl_module.' prefix
from .mcl import EnvironmentMap, MonteCarloLocalization # Adjusted path assuming subfolder

# --- Keep other imports if needed ---
from .trajectory import Trajectory
# from .icp import icp # Assuming not directly needed
# from .utils import * # Assuming not directly needed
from .block_vison_camera import *
from .config import config
from .obstacle import Obstacle
from .lidar import lidar_to_obstacles
from .path_planing import generate_trajectories
# from .block_vison_camera import get_nearest_block_data # Already imported via *

# === Paste the MCL classes here if not importing ===
# class EnvironmentMap: ...
# class Particle: ...
# class MonteCarloLocalization: ...
# ===============================================

# --- Constants from MCL (adjust as needed) ---
# Map Definition (ensure these match your EnvironmentMap class if defined separately)
MAP_SIZE_X = 3.0
MAP_SIZE_Y = 3.0
OBSTACLE_CENTER_X = 1.5
OBSTACLE_CENTER_Y = 1.5
OBSTACLE_SIZE_X = 1.0
OBSTACLE_SIZE_Y = 1.0
# Robot/Sensor Parameters
LIDAR_FOV = np.radians(200.0) # Use radians internally
LIDAR_NUM_BEAMS_MCL = 20 # Number of beams used *by MCL* (can be different from raw scan)
LIDAR_MAX_RANGE = 3.0 # Max range considered *by MCL* (<= physical max range)
LIDAR_STD_DEV = 0.15  # Tunable: Standard deviation of LIDAR sensor noise for MCL weighting
ROBOT_WHEELBASE = 0.16 # Tunable: Example wheelbase in meters (MUST BE ACCURATE)
# MCL Parameters
NUM_PARTICLES = 300 # Tunable
# Tunable: Noise for MCL prediction step
MOTION_NOISE_STD_DEV = {'vel': 0.1, 'steer': np.radians(5.0), 'odom': 0.05}
RESAMPLE_THRESHOLD = 0.5
# Steering Angle Estimation
MAX_STEERING_ANGLE_RAD = np.radians(30.0) # Tunable: Max steering angle corresponding to direction = +/- 1

# --- End Constants ---


class OjbectTrackingNode(Node):
    def __init__(self, name):
        # Use __node_name variable for consistency if needed elsewhere
        self.__node_name = name
        # Initialize ROS node
        # rclpy.init() # Should not be called inside the Node constructor
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.get_logger().info(f"Initializing node '{self.__node_name}'...")

        # self.name = name # Already set by super().__init__
        self.set_callback = False
        self.color_picker = None
        self.tracker = None
        self.is_running = False
        # self.threshold = 0.1 # Related to object tracking? Keep if needed
        # self.dist_threshold = 0.3 # Related to object tracking? Keep if needed
        self.lock = threading.RLock() # General lock
        self.image_sub = None
        self.result_image = None
        self.image_height = None
        self.image_width = None
        self.bridge = CvBridge()

        # --- Publishers ---
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        # Optional: Publisher for estimated Odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom_mcl', 10)

        # --- Subscriptions ---
        #self.image_sub = self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 1)

        # Configure QoS for Lidar subscription - BEST_EFFORT can lose scans, consider RELIABLE if needed
        # Using RELIABLE for localization is generally safer if network allows. Keep history small.
        # Using VOLATILE durability as we only care about the latest scan for MCL update.
        lidar_qos = QoSProfile(
             depth=1,
             reliability=QoSReliabilityPolicy.BEST_EFFORT, # Or RELIABLE
             durability=QoSDurabilityPolicy.VOLATILE)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, lidar_qos)

        # --- State Variables ---
        # Pose estimated by MCL
        self.position = np.array([0.5, 0.5]) # Initial estimated position (e.g., near corner)
        self.rotation = np.radians(45.0) # Initial estimated orientation
        self.pose_lock = threading.Lock() # Lock specifically for pose variables
        self.position_history = [] # Keep if needed for plotting paths

        # Last commanded/intended controls (used for MCL prediction)
        self.current_velocity = 0.0
        self.current_steering_angle = 0.0 # In radians

        # Timing for MCL prediction
        self.last_mcl_update_time = self.get_clock().now()

        # --- MCL Initialization ---
        self.get_logger().info("Initializing Monte Carlo Localization...")
        self.env_map = EnvironmentMap() # Uses constants defined above

        # Define the angles MCL expects for measurements (subset of LIDAR_FOV)
        self.mcl_lidar_angles = np.linspace(-LIDAR_FOV / 2, LIDAR_FOV / 2, LIDAR_NUM_BEAMS_MCL)

        # Create MCL instance
        try:
             self.mcl = MonteCarloLocalization(
                 env_map=self.env_map,
                 num_particles=NUM_PARTICLES,
                 wheelbase=ROBOT_WHEELBASE,
                 motion_noise_std=MOTION_NOISE_STD_DEV,
                 sensor_std_dev=LIDAR_STD_DEV,
                 lidar_angles=self.mcl_lidar_angles
             )
             # Set initial pose estimate from MCL's initial particle distribution center
             initial_est_x, initial_est_y, initial_est_theta = self.mcl.estimate_pose()
             with self.pose_lock:
                 self.position = np.array([initial_est_x, initial_est_y])
                 self.rotation = initial_est_theta
             self.get_logger().info(f"MCL Initialized. Initial Estimated Pose: ({initial_est_x:.2f}, {initial_est_y:.2f}, {np.degrees(initial_est_theta):.1f} deg)")
        except Exception as e:
             self.get_logger().error(f"Failed to initialize MCL: {e}")
             # Handle error appropriately - maybe shut down or run without MCL
             self.mcl = None


        # --- Other Variables ---
        # self.goal = np.array([0,1]) # Keep if used by DWA/other planners
        self.i = 0 # Image callback counter

        # --- Visualization Setup ---
        self.enable_visualization = True # Set to False to disable plotting
        if self.enable_visualization:
            self.get_logger().info("Initializing Plotting...")
            plt.figure() # Create figure for animation
            plt.ion() # Turn on interactive mode
            plt.show() # Show the plot window

        self.get_logger().info('\033[1;32mNode initialized successfully.\033[0m')


    def log(self, msg):
        self.get_logger().info('\033[1;32m%s\033[0m' % msg)

    # get_node_state seems unused, keep if needed
    # def get_node_state(self, request, response):
    #     response.success = True
    #     return response

    def publish_servo_state(self, positions):
        # Ensure positions are integers
        int_positions = [int(p) for p in positions]
        servo_state = PWMServoState()
        servo_state.id = [3] # Assuming servo ID 3 for steering
        servo_state.position = int_positions # Use integer positions
        data = SetPWMServoState()
        data.state = [servo_state]
        data.duration = 0.02 # Use float for duration
        # self.get_logger().info(f'Publishing servo state: {data}') # Can be verbose
        self.servo_state_pub.publish(data)

    # REMOVED update_location - MCL handles pose updates

    def steer(self, direction):
        """
        Sets the steering command based on direction (-1 to 1).
        Updates self.current_steering_angle for MCL prediction.
        """
        with self.lock: # Use general lock or a specific control lock
            direction = max(-1.0, min(1.0, float(direction))) # Ensure float and clamp

            # --- Estimate steering angle for MCL ---
            # Simple linear mapping: direction * max_angle
            self.current_steering_angle = direction * MAX_STEERING_ANGLE_RAD
            # self.get_logger().info(f"Commanded steer direction: {direction:.2f} -> angle: {np.degrees(self.current_steering_angle):.1f} deg")

            # --- Publish Servo Command ---
            # Convert direction to servo value (adjust range as needed)
            max_steer_offset = 600 # Servo range offset
            norm_dir_servo = 1500 # Servo center value
            new_angle_servo = norm_dir_servo + direction * max_steer_offset
            self.publish_servo_state([int(new_angle_servo)]) # Send command to servo

        # DO NOT CALL update_location() here

    def set_speed(self, speed):
        """
        Sets the linear velocity command.
        Updates self.current_velocity for MCL prediction.
        speed: -1 to 1 (mapping to actual m/s depends on robot base)
               -> Let's assume 'speed' directly corresponds to m/s for MCL, adjust if needed
        """
        with self.lock: # Use general lock or a specific control lock
            # Assuming speed argument IS the desired velocity in m/s
            # Clamp if necessary based on robot limits
            actual_velocity = max(config.min_speed, min(config.max_speed, float(speed)))
            self.current_velocity = actual_velocity
            # self.get_logger().info(f"Commanded velocity: {self.current_velocity:.2f} m/s")

            # --- Publish Twist Command ---
            t = Twist()
            # Use the *intended* velocity for the command, even if MCL uses a slightly different internal value
            # The sign convention might need adjustment based on your controller setup
            t.linear.x = float(speed) # Send original requested speed value
            self.mecanum_pub.publish(t)

        # DO NOT CALL update_location() here

    # --- DWA Related Methods (Keep if DWA is still used alongside MCL) ---
    # These functions might need to be adapted to use the MCL pose estimate
    def calculate_dynamic_window(self):
        """
        Calculates the dynamic window based on CURRENT estimated state.
        """
        with self.pose_lock:
            current_vel = self.current_velocity # Or use velocity derived from MCL particles if desired
            current_omega = 0 # How to get current omega? Estimate from steering angle?
            # Rough estimate: omega = v * tan(steer) / L
            if abs(self.current_velocity) > 1e-3:
                 current_omega = self.current_velocity * tan(self.current_steering_angle) / ROBOT_WHEELBASE
            else:
                 current_omega = 0.0

        # Fetch dt based on last MCL update interval
        # This might not be ideal for DWA which needs faster prediction?
        # Consider calculating dt more frequently if DWA runs faster than lidar scans
        # For now, use last MCL dt
        # dt = ... (Need a reliable dt for DWA prediction horizon)
        # Placeholder dt
        dt_dwa = config.predict_time / config.dt # Example: use DWA's internal dt

        # Dynamic window from robot specification
        Vs = [config.min_speed, config.max_speed,
            -config.max_yaw_rate, config.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [current_vel - config.max_accel * dt_dwa,
            current_vel + config.max_accel * dt_dwa,
            current_omega - config.max_delta_yaw_rate * dt_dwa,
            current_omega + config.max_delta_yaw_rate * dt_dwa]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    def obstacle_cost(self, trajectory: Trajectory, obstacles: list): # Assuming obstacles list
        """Calculates the cost related to obstacle proximity for a trajectory."""
        min_dist = float('inf')
        for obs in obstacles:
             dist = obs.get_distance(trajectory) # Assuming Obstacle class has this method
             if dist < min_dist:
                 min_dist = dist

        # Cost calculation: Higher cost for closer obstacles
        if min_dist < config.obstacle_radius: # Define a safety radius
            return float('inf') # Collision cost
        elif min_dist > config.obstacle_radius + config.safe_dist: # Define safe distance buffer
            return 0.0 # No cost if far enough
        else:
            # Scale cost inversely with distance within the buffer zone
            return (config.safe_dist - (min_dist - config.obstacle_radius)) / config.safe_dist

    def goal_cost(self, trajectory, goal):
        """Calculates the cost related to reaching the goal for a trajectory."""
        # Cost based on distance to goal at end of trajectory
        final_pos = trajectory.positions[-1, 0:2] # Assuming positions attribute exists
        dist_to_goal = np.linalg.norm(final_pos - goal)

        # Cost based on heading error at end of trajectory
        dx = goal[0] - final_pos[0]
        dy = goal[1] - final_pos[1]
        error_angle = atan2(dy, dx)
        final_heading = trajectory.positions[-1, 2] # Assuming heading is stored at index 2
        cost_angle = abs(atan2(sin(error_angle - final_heading), cos(error_angle - final_heading)))

        # Combine costs (example weighting)
        cost = config.goal_dist_cost_gain * dist_to_goal + config.goal_angle_cost_gain * cost_angle
        return cost

    def velocity_cost(self, trajectory):
        """Penalizes low velocities to encourage progress."""
        # Example: Prefer higher velocities up to a target speed
        target_speed = config.max_speed * 0.8
        vel_cost = target_speed - trajectory.v # Assuming v attribute stores trajectory velocity
        return max(0, vel_cost) # Only penalize if below target


    # predict_trajectory seems unused, keep if needed by DWA
    # def predict_trajectory(self, v, yaw) -> Trajectory: ...

    def select_best_trajectory(self, trajectories, obstacles, goal):
        """Evaluates trajectories and selects the best one based on costs."""
        min_cost = float('inf')
        best_traj = None

        for traj in trajectories:
            obs_cost = self.obstacle_cost(traj, obstacles) * config.obstacle_cost_gain
            goal_cost_val = self.goal_cost(traj, goal) # Already includes gains inside
            vel_cost = self.velocity_cost(traj) * config.velocity_cost_gain

            total_cost = obs_cost + goal_cost_val + vel_cost

            if obs_cost == float('inf'): # Skip trajectories that collide
                continue

            if total_cost < min_cost:
                min_cost = total_cost
                best_traj = traj

        return best_traj

    # --- End DWA Methods ---


    def image_callback(self, ros_image):
        # Only process image if MCL is running
        if not self.mcl:
             self.get_logger().warn("MCL not initialized, skipping image processing.")
             return

        # Image processing for object detection/tracking remains largely the same
        # However, control commands issued here (`steer`, `set_speed`) will now
        # only update the *intended* state for the *next* MCL prediction step.
        self.i += 1
        # self.log(f"Startup: {self.i}")
        if self.i < 20: # Initial delay might still be useful
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
            rgb_image = np.array(cv_image, dtype=np.uint8)
            if self.image_height is None: # Get dimensions once
                self.image_height, self.image_width = rgb_image.shape[:2]
            # self.get_logger().info('Image callback') # Can be verbose

            # --- Your existing image processing ---
            rgb_image = cv.cvtColor(rgb_image, cv.COLOR_BGR2RGB)
            rgb_image_cropped = rgb_image[210:480, :] # Use a different variable name

            res = get_nearest_block_data(rgb_image_cropped) # Pass cropped image
            color, (centerX_rel, centerY_rel), (width, height), processed_image = res

            # Normalize coordinates based on the *cropped* image dimensions if needed
            # Or ensure get_nearest_block_data returns coordinates relative to original if necessary
            # Assuming centerX_rel, centerY_rel are relative to the *original* image width/height
            # If they are relative to cropped image, adjust calculation
            if self.image_width > 0 and self.image_height > 0: # Avoid division by zero
                 centerX_norm = centerX_rel / self.image_width
                 centerY_norm = (centerY_rel + 210) / self.image_height # Adjust for crop offset
            else:
                 centerX_norm = 0.5
                 centerY_norm = 0.5


            if self.enable_visualization:
                 cv.imshow("Processed Image", processed_image) # Show the processed part
                 cv.waitKey(1)
            # --- End image processing ---


            # --- Control Logic based on vision ---
            if color:
                target_speed = 0.05 # Slow approach speed
                self.set_speed(target_speed) # Set desired speed for next MCL step

                # Simple proportional steering based on object centering and width
                if width < 50:
                    goalX_norm = 0.5 # Center target
                    diff = goalX_norm - centerX_norm
                    steer_cmd = diff * 2.0 # Proportional gain (tune)
                    self.steer(steer_cmd)
                    # self.log(f"Steering (far): diff={diff:.2f}, cmd={steer_cmd:.2f}")
                elif width < 110:
                    goalX_norm = 0.8 # Target slightly to the side? Adjust logic
                    diff = goalX_norm - centerX_norm
                    steer_cmd = diff * 1.5 # Proportional gain (tune)
                    self.steer(steer_cmd)
                    # self.log(f"Steering (mid): diff={diff:.2f}, cmd={steer_cmd:.2f}")
                else: # Object is close and wide
                    # Stop or perform final maneuver?
                    self.log(f"Object close (width {width}). Stopping.")
                    self.set_speed(0.0)
                    self.steer(0.0) # Straighten wheels
            else:
                # No object detected, maybe stop or search?
                self.set_speed(0.0)
                self.steer(0.0)
                # self.log("No object detected.")

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")


    def plot_lidar(self, lidar_data):
        self.log("Lidar callback")
 
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
        ox = -np.sin(angles) * distances
        scan = np.column_stack((ox, oy))
        
        plt.cla()
        plt.grid(True)
        plt.xlim((3, -3)) # rescale y axis, to match the grid orientation
        plt.ylim((3, -3)) # rescale y axis, to match the grid orientation
        plt.scatter(ox, oy, s=2, c="blue") # lines from 0,0 to the
        plt.xlabel("X")
        plt.ylabel("Y")

        plt.pause(0.0001)

 
        self.last_laser_scan = scan
         

    def lidar_callback(self, lidar_data):
        self.plot_lidar(lidar_data)
        
        
        # self.get_logger().info('Lidar callback') # Can be very verbose

        # Only process lidar if MCL is running
        if not self.mcl:
             # self.get_logger().warn("MCL not initialized, skipping lidar processing.")
             return

        current_time = self.get_clock().now()
        dt_duration = current_time - self.last_mcl_update_time
        dt = dt_duration.nanoseconds / 1e9 # Get dt in seconds

        # Avoid huge dt on first callback or after delays
        if dt > 1.0:
             self.get_logger().warn(f"Large dt detected ({dt:.2f}s), capping predict step dt to 0.1s")
             predict_dt = 0.1
        else:
             predict_dt = dt
        self.last_mcl_update_time = current_time

        # --- 1. MCL Predict Step ---
        # Use the *last commanded* velocity and steering angle
        with self.lock: # Access commanded state safely
            predict_velocity = self.current_velocity
            predict_steering_angle = self.current_steering_angle
        # Run prediction using the time elapsed since last update
        self.mcl.predict(predict_velocity, predict_steering_angle, predict_dt)
        # self.get_logger().info(f"MCL Predict: v={predict_velocity:.2f}, steer={np.degrees(predict_steering_angle):.1f}, dt={predict_dt:.3f}")


        # --- 2. Process LaserScan for MCL Update ---
        raw_ranges = np.array(lidar_data.ranges)
        raw_angles = np.linspace(lidar_data.angle_min, lidar_data.angle_max, len(raw_ranges))

        # Select measurements corresponding to MCL's expected angles
        # Use nearest neighbor lookup for simplicity
        measurement_array = np.zeros_like(self.mcl_lidar_angles)
        for i, mcl_angle in enumerate(self.mcl_lidar_angles):
            # Find the index in raw_angles closest to mcl_angle
            # Assumes mcl_angle is relative to robot forward, and lidar 0 angle is also forward
            # If lidar has an offset, adjust mcl_angle lookup accordingly
            target_angle = mcl_angle # Adjust if lidar frame != robot frame base_link forward
            
            # Check if target_angle is within the lidar's scan range
            if target_angle < lidar_data.angle_min or target_angle > lidar_data.angle_max:
                measurement_array[i] = LIDAR_MAX_RANGE # Or some other default value for out-of-range
                continue

            idx = np.argmin(np.abs(raw_angles - target_angle))
            dist = raw_ranges[idx]

            # Clean up the measurement
            if np.isnan(dist) or np.isinf(dist) or dist < lidar_data.range_min:
                dist = LIDAR_MAX_RANGE # Use max range for invalid readings
            elif dist > LIDAR_MAX_RANGE: # Clamp to MCL's max range
                dist = LIDAR_MAX_RANGE

            measurement_array[i] = dist
        # self.get_logger().info(f"MCL Measurement (first 5): {measurement_array[:5]}")


        # --- 3. MCL Update Step ---
        self.mcl.update(measurement_array)

        # --- 4. MCL Resample Step ---
        self.mcl.resample()

        # --- 5. Get Estimated Pose ---
        est_x, est_y, est_theta = self.mcl.estimate_pose()
        with self.pose_lock:
            self.position[0] = est_x
            self.position[1] = est_y
            self.rotation = est_theta
            current_position = self.position.copy() # Get copies for use outside lock
            current_rotation = self.rotation
        # self.get_logger().info(f"MCL Estimate: x={est_x:.2f}, y={est_y:.2f}, theta={np.degrees(est_theta):.1f}")

        # --- 6. Publish Odometry Message (Optional) ---
        self.publish_odom_message(current_position, current_rotation, current_time)


        # --- 7. Visualization (Optional) ---
        if self.enable_visualization:
             self.plot_mcl_state(lidar_data, measurement_array) # Pass processed data if needed

        # --- 8. Path Planning / Control (e.g., DWA) ---
        # If using DWA or another planner, it should run here using the *updated* pose
        # goal = np.array([0,1]) # Example fixed goal
        # goal_relative_to_map = goal # Adjust if goal is in different frame

        # # Generate obstacle list from current lidar (if using DWA obstacle cost)
        # scan_points = self.lidar_to_points(lidar_data) # Helper function needed
        # obstacles_list = lidar_to_obstacles(scan_points) # Your existing function

        # # Generate trajectories from current *estimated* state
        # with self.pose_lock:
        #      dwa_trajectories = generate_trajectories(
        #          current_position,
        #          current_rotation,
        #          self.current_velocity, # Use current commanded velocity as basis?
        #          self.current_steering_angle # Or omega derived from it?
        #      )

        # # Select best trajectory
        # best_dwa_trajectory = self.select_best_trajectory(dwa_trajectories, obstacles_list, goal_relative_to_map)

        # # Extract control commands from best trajectory
        # if best_dwa_trajectory:
        #      # Convert trajectory commands (v, omega) to your robot's commands (speed, steer direction)
        #      # This conversion needs care!
        #      dwa_v = best_dwa_trajectory.v
        #      dwa_omega = best_dwa_trajectory.omega
        #      # Example conversion (NEEDS VERIFICATION/TUNING):
        #      # Set speed directly
        #      self.set_speed(dwa_v)
        #      # Convert omega back to steering angle/direction
        #      if abs(dwa_v) > 1e-2:
        #           dwa_steer_angle = atan2(dwa_omega * ROBOT_WHEELBASE, dwa_v)
        #           # Convert angle back to direction (-1 to 1)
        #           steer_direction = np.clip(dwa_steer_angle / MAX_STEERING_ANGLE_RAD, -1.0, 1.0)
        #           self.steer(steer_direction)
        #      else:
        #           self.steer(0.0) # Steer straight if not moving fast
        #      self.log(f"DWA Control: v={dwa_v:.2f}, omega={dwa_omega:.2f} -> steer={steer_direction:.2f}")
        # else:
        #      # No valid trajectory found by DWA
        #      self.log("DWA: No valid trajectory found. Stopping.")
        #      self.set_speed(0.0)
        #      self.steer(0.0)


    def lidar_to_points(self, lidar_data):
        """Helper to convert LaserScan to point cloud (x, y) in sensor frame."""
        ranges = np.array(lidar_data.ranges)
        angles = np.linspace(lidar_data.angle_min, lidar_data.angle_max, len(ranges))

        # Apply range limits and filter invalid readings
        ranges[ranges < lidar_data.range_min] = np.inf # Or lidar_data.range_max
        ranges[ranges > lidar_data.range_max] = np.inf # Or lidar_data.range_max
        valid_indices = np.isfinite(ranges)

        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        # Convert polar to Cartesian (assuming standard ROS coordinate frames: x forward, y left)
        # Adjust if your sensor frame is different
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        points = np.column_stack((x, y))
        return points


    def publish_odom_message(self, position, rotation, timestamp):
        """Publishes the estimated pose as an Odometry message."""
        odom_msg = Odometry()
        #odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = "odom" # Or your relevant odometry frame
        odom_msg.child_frame_id = "base_link" # Or your robot's base frame

        # Set pose
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = 0.0 # Assuming 2D

        # Convert yaw rotation to quaternion
        qx, qy, qz, qw = self.yaw_to_quaternion(rotation)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Set covariance (example: higher uncertainty in y and rotation)
        # Adjust these based on expected MCL performance
        odom_msg.pose.covariance[0] = 0.1  # x variance
        odom_msg.pose.covariance[7] = 0.1  # y variance
        odom_msg.pose.covariance[14] = 1e6 # z variance (large)
        odom_msg.pose.covariance[21] = 1e6 # roll variance (large)
        odom_msg.pose.covariance[28] = 1e6 # pitch variance (large)
        odom_msg.pose.covariance[35] = 0.2  # yaw variance

        # Twist is optional - could estimate from pose change, but often set to zero if not measured
        odom_msg.twist.twist.linear.x = self.current_velocity # Use commanded velocity as estimate
        # Estimate omega from steering angle (rough)
        if abs(self.current_velocity) > 1e-3:
            current_omega = self.current_velocity * tan(self.current_steering_angle) / ROBOT_WHEELBASE
        else:
            current_omega = 0.0
        odom_msg.twist.twist.angular.z = current_omega
        odom_msg.twist.covariance[0] = 0.2 # linear x variance
        odom_msg.twist.covariance[7] = 1e6 # linear y variance
        odom_msg.twist.covariance[35] = 0.3 # angular z variance

        self.odom_pub.publish(odom_msg)

    @staticmethod
    def yaw_to_quaternion(yaw):
        """Converts a Yaw angle (radians) to a Quaternion."""
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        return 0.0, 0.0, sy, cy # (x, y, z, w)

    def plot_mcl_state(self, lidar_data, processed_ranges):
         """Visualizes the MCL state using matplotlib."""
         try:
             plt.clf() # Clear the current figure

             # --- Plot Map ---
             self.env_map.plot() # Add a plot method to EnvironmentMap if it doesn't exist

             # --- Plot Particles ---
             if self.mcl:
                 px = [p.x for p in self.mcl.particles]
                 py = [p.y for p in self.mcl.particles]
                 pw = np.array([p.weight for p in self.mcl.particles])
                 # Normalize weights for color/size scaling if needed
                 pw_norm = pw / (np.max(pw) + 1e-9)
                 particle_colors = plt.cm.viridis(pw_norm) # Color by weight

                 plt.scatter(px, py, color=particle_colors, s=10, label="Particles", alpha=0.6, zorder=3)

             # --- Plot Estimated Pose ---
             with self.pose_lock:
                 est_x, est_y, est_theta = self.position[0], self.position[1], self.rotation
             plt.plot(est_x, est_y, 'bo', markersize=8, label="Estimated Pose", zorder=5)
             plt.arrow(est_x, est_y,
                       0.3 * cos(est_theta), 0.3 * sin(est_theta),
                       head_width=0.08, head_length=0.12, fc='b', ec='b', zorder=5)

             # --- Plot Processed LIDAR used for MCL ---
             # Transform processed_ranges back to points relative to estimated pose
             lidar_angles_for_plot = self.mcl_lidar_angles + est_theta # World angles
             lx = est_x + processed_ranges * np.cos(lidar_angles_for_plot)
             ly = est_y + processed_ranges * np.sin(lidar_angles_for_plot)
             plt.scatter(lx, ly, color='orange', s=5, label="MCL Scan Points", zorder=2, alpha=0.8)


             # --- Plot DWA info if available ---
             # Example: Plot best trajectory if computed
             # if best_dwa_trajectory:
             #     best_dwa_trajectory.plot(color='g', label='Best DWA Traj', zorder=4)


             plt.title(f"MCL State - Step Time: {self.last_mcl_update_time.nanoseconds/1e9:.2f}")
             plt.xlabel("X (m)")
             plt.ylabel("Y (m)")
             plt.axis("equal")
             plt.xlim(-0.5, MAP_SIZE_X + 0.5) # Adjust limits as needed
             plt.ylim(-0.5, MAP_SIZE_Y + 0.5)
             plt.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0), fontsize='small')
             plt.grid(True)
             plt.tight_layout(rect=[0, 0, 0.85, 1]) # Adjust layout for legend

             plt.pause(0.01) # Allow plot to update

         except Exception as e:
             self.get_logger().error(f"Plotting error: {e}")


# --- Add plot method to EnvironmentMap if needed ---
# Example:
def plot_map(self): # Add this method to your EnvironmentMap class
    plt.plot([0, self.width, self.width, 0, 0], [0, 0, self.height, self.height, 0], 'k-', linewidth=2, label="Walls", zorder=1)
    obstacle = plt.Rectangle((self.obstacle_min_x, self.obstacle_min_y),
                         self.obstacle_max_x - self.obstacle_min_x,
                         self.obstacle_max_y - self.obstacle_min_y,
                         facecolor='gray', edgecolor='k', linewidth=2, label="Obstacle", zorder=1)
    plt.gca().add_patch(obstacle)
# Monkey-patch it if EnvironmentMap is imported and cannot be modified:
# EnvironmentMap.plot = plot_map
# Make sure to call this *after* importing EnvironmentMap


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        # --- Monkey-patch the plot method onto the imported class ---
        # Ensure this happens before node instantiation if EnvironmentMap needs it
        if 'EnvironmentMap' in globals() or 'EnvironmentMap' in locals():
             EnvironmentMap.plot = plot_map
        # --- End Monkey-patch ---

        node = OjbectTrackingNode('object_tracking_mcl') # Renamed node
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C detected, shutting down.")
    except Exception as e:
        if node:
             node.get_logger().fatal(f"Unhandled exception: {e}")
        else:
             print(f"Exception during node creation: {e}")
    finally:
        if node:
            # Cleanup matplotlib
            if node.enable_visualization:
                plt.ioff()
                plt.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()