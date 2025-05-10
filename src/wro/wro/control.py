# ros uses relative imports
import sys
import time
import numpy as np


try:
    from .lidar import LidarResult
except:
    from lidar import LidarResult

"""
Important Note:
stearing: 
0 = front
-1 = left
+1 = right

"""
class Control:
    def __init__(self, logger=None):
        self.lidar_res = LidarResult()
        self.state = "MOVING"
        
        self.logger = None
        if logger is not None:
            self.logger = logger
            
        """
        TURNING variables
        """
        # RIGHT | LEFT
        self.turning_direction = ""
        
        self.time_start = 0
     
    def log(self, msg):
        if self.logger:
            self.logger.info('\033[1;32m%s\033[0m' % msg)
        else:
            print(msg)
        
    def update_lidar(self, raw_data):
        self.lidar_res = LidarResult(raw_data)

    def update_odom(self, odom_data):
        pass
    
    def change_state(self, new_state):
        if new_state not in ["MOVING", "TURNING"]:
            raise ValueError(f"Invalid state: {new_state}")
        
        self.log(f"STATE CHANGE: {self.state} -> {new_state}")

        if new_state == "TURNING":
            distance_left = self.lidar_res.get_distance_for_angle(-np.pi/2)
            distance_right = self.lidar_res.get_distance_for_angle(np.pi/2)
            
            if self.turning_direction == "":
                if distance_left < distance_right:
                    self.log("TURNING: DIRECTION CALCULATED: LEFT")
                    self.turning_direction = "LEFT"
                else:
                    self.log("TURNING: DIRECTION CALCULATED: RIGHT")
                    self.turning_direction = "RIGHT"
                
        self.state = new_state
    
    def get_control_strategy(self):
        """
        Returns speed and steering direction
        """
        if self.time_start == 0:
            self.time_start = time.time()
        
        speed = 0.5
        stear = 0
        """
        ---------------------------------------
        MOVING Strategy
        ---------------------------------------
        """
        MIN_DISTANCE_TO_WALL = 0.8

        if self.state == "MOVING":
            stear = 0
            
            """
            Strategy change
            """
            distance_to_front = self.lidar_res.get_distance_for_angle(0)
            self.log(f"MOVING: Distance: F{distance_to_front:.3f}")

            if distance_to_front < MIN_DISTANCE_TO_WALL:
                self.change_state("TURNING")

        
        """
        ---------------------------------------
        TURNING Strategy
        ---------------------------------------
        """
        TURN_UNTIL_DISTANCE_GREATER = 2
        TURN_UNTIL_ANGLE_SMALLER = 0.2
        
        if self.state == "TURNING":
            angle_to_dir = 0
            distance_to_dir = 0
            if self.turning_direction == "RIGHT":
                stear = 1
                angle_to_dir = self.lidar_res.get_angle_to_wall(np.pi / 2)
                distance_to_dir = self.lidar_res.get_median_distance_for_angle(np.pi / 2)
            elif self.turning_direction == "LEFT":
                stear = -1
                angle_to_dir = self.lidar_res.get_angle_to_wall(-np.pi / 2)
                distance_to_dir = self.lidar_res.get_median_distance_for_angle(-np.pi / 2)
                
            med_distance_to_front = self.lidar_res.get_median_distance_for_angle(0)
            angle_to_front = self.lidar_res.get_angle_to_wall(0)
        
            self.log(f"TURNING: Angle: F:{angle_to_front:.3f} D:{angle_to_dir:.3f} | Distance: F:{med_distance_to_front:.3f} D:{distance_to_dir:.3f}")
            if angle_to_front < TURN_UNTIL_ANGLE_SMALLER:
                self.change_state("MOVING")

        self.log(f"SPEED: {speed} STEAR: {stear}")
        
        if abs(self.time_start - time.time()) > 52:
            speed = 0
        
        return speed, stear