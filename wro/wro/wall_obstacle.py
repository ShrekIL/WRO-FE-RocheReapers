from matplotlib import pyplot as plt
import numpy as np


class WallObstacle:
    """
    Class representing a wall obstacle in a 2D space.
    """

    def __init__(self, start, end):
        """
        Initialize the WallObstacle with start and end points.

        :param start: Tuple representing the start point [x, y].
        :param end: Tuple representing the end point [x, y].
        """
        self.start = np.array(start)
        self.end = np.array(end)

    def __repr__(self):
        return f"WallObstacle(start={self.start}, end={self.end})"
    
    def distance(self, point):
        """
        Calculate the distance from a point to the wall.

        :param point: Tuple representing the point [x, y].
        :return: Distance from the point to the wall.
        """
        P = np.array(point)
        A = self.start
        B = self.end
        

        AP = P - A
        AB = B - A
        
        # Project point P onto line AB, but consider segment constraints
        AB_squared = np.dot(AB, AB)
        if AB_squared == 0:
            return np.linalg.norm(AP)  # A and B are the same point
        
        t = np.dot(AP, AB) / AB_squared
        t = np.clip(t, 0, 1)  # Ensure the projection lies on the segment
        
        closest_point = A + t * AB
        return np.linalg.norm(P - closest_point)

    
    def plot(self, color='blue'):
        """
        Plot the wall obstacle on a given axis.

        :param ax: Matplotlib axis to plot on.
        :param color: Color of the wall.
        """
        plt.plot([self.start[0], self.end[0]], [self.start[1], self.end[1]], color=color)