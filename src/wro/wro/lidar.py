import time
from matplotlib import pyplot as plt
import numpy as np

"""
Important note:
0 deg = front
90 deg = right
180 deg = back
270 deg = left


cartesian coordinates:
to left = x < 0
to right = x > 0
front = y > 0
back = y < 0
"""


class LidarResult:
    def __init__(self, raw_data=None):
        if not raw_data:
            self.angles = np.array([])
            self.ranges = np.array([])
        else:
            self.angles = np.linspace(raw_data.angle_min, raw_data.angle_max, len(raw_data.ranges))

            ranges = np.array(raw_data.ranges)
            ranges = np.where(np.isnan(ranges), 0, ranges)
            ranges = np.where(np.isinf(ranges), 0, ranges)
            ranges = np.where(ranges > 3, 0, ranges)
            ranges = np.where(ranges < 0.1, 0, ranges)

            mask = ranges != 0

            self.angles = self.angles[mask]
            
            # turn pi - because lidar is mounted backwards
            self.angles = self.angles + np.pi
            self.correct_angles()
            
            self.ranges = ranges[mask]
            
    def correct_angles(self):
        """
        Clips all angles between 0 and 2pi
        """

        mask = self.angles < 0
        self.angles[mask] = self.angles[mask] + np.pi * 2

        mask = self.angles > np.pi * 2
        self.angles[mask] = self.angles[mask] - np.pi * 2

    @staticmethod
    def _correct_single_angle(angle):
        while angle < 0:
            angle += np.pi * 2
        while angle > np.pi * 2:
            angle -= np.pi * 2
        return angle

    def get_distance_for_angle(self, angle):
        """
        Returns the distance closest to the given angle using binary search.
        :param angle: Angle in radians.
        :return: Distance at the given angle.
        """
        angle = self._correct_single_angle(angle)
        
        diff = np.abs(self.angles - angle)
        index = np.argmin(diff)
        return self.ranges[index]
    
    def get_median_distance_for_angle(self, angle) -> float:
        """
        Returns the distance gotten by calculating the median between a range of angles
        This is, so it can still get the distance even if a obstacle is in front of him
        """
        
        """
        Params
        """
        # +- 0.2 rad = +- 11deg
        window = 0.2
        num_points = 15
        
        
        self._correct_single_angle(angle)


        distances = []
        for i in range(num_points):
            scan_angle = angle - ((i / (num_points - 1)) * 2 - 1) * window
            distances.append(self.get_distance_for_angle(scan_angle))
        
        return float(np.median(np.array(distances)))
    
    def get_angle_to_wall(self, angle) -> float:
        """
        USe SVD to get the angle to a wall
        
        **Warning**: returns an angle between -pi/2 and pi/2
        """
        
        """
        params
        """
        # +- 11deg
        window = 0.2
        num_points = 15
        angle = self._correct_single_angle(angle)
        
        selected_angles = []
        selected_ranges = []
        for i in range(num_points):
            scan_angle = angle - ((i / (num_points - 1)) * 2 - 1) * window
            selected_ranges.append(self.get_distance_for_angle(scan_angle))
            selected_angles.append(scan_angle)
        
        selected_angles = np.array(selected_angles)
        selected_ranges = np.array(selected_ranges)

        if len(selected_angles) < 2:
            raise ValueError("Not enough points to estimate wall angle.")

        # Convert polar to Cartesian
        xs = selected_ranges * np.cos(selected_angles)
        ys = selected_ranges * np.sin(selected_angles)

        # Fit line using PCA (principal component analysis)
        coords = np.vstack((xs, ys)).T
        coords_mean = np.mean(coords, axis=0)
        centered = coords - coords_mean
        _, _, vh = np.linalg.svd(centered)
        direction_vector = vh[0]  # first principal component

        # Get angle of wall relative to robot frame
        wall_angle = np.arctan2(direction_vector[1], direction_vector[0])

        if wall_angle < 0:
            wall_angle += np.pi

        wall_angle -= np.pi / 2
        wall_angle += angle
        
        if wall_angle > np.pi:
            wall_angle -= np.pi

        return wall_angle

    
    def get_cartesian_for_angle(self, angle):
        """
        Convert polar coordinates (ranges and angles) to Cartesian coordinates (x, y).
        
        :return: Tuple (x, y) representing the Cartesian coordinates.
        """
        angle = self._correct_single_angle(angle)
       
        dist = self.get_distance_for_angle(angle)

        # Convert polar coordinates to Cartesian coordinates
        x = dist * np.cos(angle)
        y = dist * np.sin(angle)

        return (x, y)
    
    def plot(self):
        plt.figure(figsize=(8, 8))
        plt.subplot(111, projection='polar')
        plt.scatter(self.angles, self.ranges, c='r', s=10)
        plt.title("Lidar Data")
        plt.xlabel("Angle (radians)")
        plt.ylabel("Distance (m)")
        plt.grid(True)
        plt.show()
    
    def get_cartesian(self):
        oy = np.cos(self.angles) * self.ranges
        ox = np.sin(self.angles) * self.ranges

        return ox, oy
        
    def plot_cartesian(self):
        ox, oy = self.get_cartesian()
        
        plt.figure(figsize=(8,8))
        plt.grid(True)
        plt.xlim((3, -3)) # rescale y axis, to match the grid orientation
        plt.ylim((-3, 3)) # rescale y axis, to match the grid orientation
        plt.scatter(ox, oy, s=2, c="blue") # lines from 0,0 to the
        plt.xlabel("X")
        plt.ylabel("Y")

        plt.show()

    
    def save(self):
        np.save("/tmp/angles.npy", self.angles)
        np.save("/tmp/ranges.npy", self.ranges)
            
    @staticmethod
    def load() -> "LidarResult":
        res = LidarResult()
        res.angles = np.load("/tmp/angles.npy")
        res.ranges = np.load("/tmp/ranges.npy")
        res.correct_angles()
        
        return res
        
if __name__ == "__main__":
    # Example usage
    lidar_data = LidarResult.load()
    lidar_data.plot()

    med_distance = lidar_data.get_median_distance_for_angle(0)
    print(f"Median Distance at 0 degrees: {med_distance}")

    distance = lidar_data.get_distance_for_angle(np.pi / 4)
    print(f"Distance at 45 degrees: {distance}")

    cartesian_coords = lidar_data.get_cartesian_for_angle(np.pi / 4)
    print(f"Cartesian coordinates at 45 degrees: {cartesian_coords}")
    
    angle_to_front = lidar_data.get_angle_to_wall(0)
    print(f"Angle to front: {angle_to_front}")
    
    angle_to_right = lidar_data.get_angle_to_wall(np.pi / 2)
    print(f"Angle to right: {angle_to_right}")

    angle_to_left = lidar_data.get_angle_to_wall(-np.pi / 2)
    print(f"Angle to left: {angle_to_left}")
    