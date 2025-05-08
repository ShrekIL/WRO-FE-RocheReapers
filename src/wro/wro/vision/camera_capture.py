import os

import numpy as np
import cv2 as cv

class RealSenseCapture:
    """
    Helper class for storing the color, depth and intrinsic matrix of a realsense image
    """
    def __init__(self, color_image: np.ndarray, depth_image: np.ndarray, intrinsic_matrix: np.ndarray):
        self.color_image: np.ndarray = color_image
        self.depth_image: np.ndarray = depth_image
        self.intrinsic_matrix: np.ndarray = intrinsic_matrix

    def get_point_in_3d_space(self, x: int, y: int) -> np.ndarray:
        """
        Get the 3D point in space from the depth image at the given x, y coordinates
        Args:
            x: X pixel coordinate in the depth image
            y: Y pixel coordinate in the depth image
        Returns:
            np.ndarray: 3D point in space
        """
        Z = self.depth_image[y, x]

        fx = self.intrinsic_matrix[0, 0]
        fy = self.intrinsic_matrix[1, 1]
        cx = self.intrinsic_matrix[0, 2]
        cy = self.intrinsic_matrix[1, 2]

        X = (x - cx) * Z / fx
        Y = (y - cy) * Z / fy

        return np.array([float(X) / 1000, float(Y) / 1000, float(Z) / 1000])

    def crop(self, x1: int, y1: int, x2: int, y2: int):
        """
        Crop the color and depth image and adjust the intrinsic matrix accordingly
        Args:
            x1: X1 coordinate of the crop
            y1: Y1 coordinate of the crop
            x2: X2 coordinate of the crop
            y2: Y2 coordinate of the crop
        """
        # test if it has already been cropped
        if x2 - x1 == self.color_image.shape[1]:
            x1 = 0
            x2 = self.color_image.shape[1]

        if y2 - y1 == self.color_image.shape[0]:
            y1 = 0
            y2 = self.color_image.shape[0]

        self.color_image = self.color_image[y1:y2, x1:x2]
        self.depth_image = self.depth_image[y1:y2, x1:x2]

        self.intrinsic_matrix[0, 2] = self.intrinsic_matrix[0, 2] - x1
        self.intrinsic_matrix[1, 2] = self.intrinsic_matrix[1, 2] - y1

    def copy(self):
        """
        Copy the color image, depth image and intrinsic matrix
        """
        return RealSenseCapture(
            self.color_image.copy(),
            self.depth_image.copy(),
            self.intrinsic_matrix.copy()
        )

    def save(self, output_path: str):
        """
        Save the color image, depth image and intrinsic matrix to the given output path
        Args:
            output_path: Path to save the images and intrinsic matrix
        """
        cv.imwrite(os.path.join(output_path, "color_image.png"), self.color_image)
        np.save(os.path.join(output_path, "intrinsic_matrix.npy"), self.intrinsic_matrix)
        np.save(os.path.join(output_path, "depth_image.npy"), self.depth_image)

    def __str__(self):
        return f"RealSenseCapture(color_image={self.color_image.shape}, depth_image={self.depth_image.shape}, intrinsic_matrix={self.intrinsic_matrix.shape})"

    def __repr__(self):
        return str(self)