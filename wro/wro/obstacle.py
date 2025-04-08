from abc import abstractmethod
import numpy as np

class Obstacle:
    def __init__(self):
        pass
        
    @abstractmethod
    def get_distance(self, pos) -> float:
        pass
    
    @abstractmethod
    def plot(self):
        pass