from matplotlib import pyplot as plt
import numpy as np


class BlockObstacle:
    """
    Class representing a wall obstacle in a 2D space.
    """

    def __init__(self, pos, color='green'):
        self.pos = np.array(pos)
        self.color = color

    def __repr__(self):
        return f"BoxObstacle({self.pos=})"
    
    def distance(self, point):
        return np.linalg.norm(self.pos - np.array(point))

    
    def plot(self):
        """
        Plot the obstacle on a 2D plane.
        """
        plt.scatter([self.pos[0]], [self.pos[1]], c="red", s=100, label='Obstacle')