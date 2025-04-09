import time
from typing import List

import numpy as np

from .block_obstacle import BlockObstacle
from .wall_obstacle import WallObstacle
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
from .obstacle import Obstacle


def lidar_to_obstacles(lidar_data) -> List[Obstacle]:
    # Consider adjusting eps based on lidar density/expected object separation
    dbscan = DBSCAN(eps=0.2, min_samples=3) # Slightly higher min_samples might reduce noise clusters
    labels = dbscan.fit_predict(lidar_data)

    unique_labels = set(labels)

    # Step 2: Fit RANSAC for each cluster and create obstacles
    lines = []
    obstacles = []

    for label in unique_labels:
        if label == -1:  # Ignore noise points labeled as -1
            continue

        cluster_points = lidar_data[labels == label]

        # Ensure enough points for RANSAC (needs at least 2)
        if len(cluster_points) < 2:
            continue

        X_cluster = cluster_points[:, 0].reshape(-1, 1)
        Y_cluster = cluster_points[:, 1]

        # Fit RANSAC to the cluster
        start_r = time.time()
        ransac = RANSACRegressor(residual_threshold=0.05) # Add a residual threshold
        ransac.fit(X_cluster, Y_cluster)
        end_r = time.time()
        #print(f"RANSAC took {end_r - start_r:.4f} seconds for cluster {label}")

        # Identify inlier points according to RANSAC
        inlier_mask = ransac.inlier_mask_
        inlier_points = cluster_points[inlier_mask]

        # Determine endpoints based on the extent of INLIERS projected onto the line
        # Get the line equation: y = mx + b or x = (y-b)/m
        m = ransac.estimator_.coef_[0]
        b = ransac.estimator_.intercept_
        line_vec = np.array([1, m]) / np.sqrt(1 + m**2) # Normalized direction vector

        # Project inlier points onto the line vector (relative to the first inlier point)
        projections = np.dot(inlier_points - inlier_points[0], line_vec)
        min_proj_idx = np.argmin(projections)
        max_proj_idx = np.argmax(projections)

        # Use the actual inlier points corresponding to min/max projection as endpoints
        p0 = inlier_points[min_proj_idx]
        p1 = inlier_points[max_proj_idx]

        # Ensure p0 and p1 are distinct enough
        length = np.linalg.norm(p1 - p0)
        # Create Obstacle based on length (use your specific classes)
        if length < 0.15: # Example threshold for 'block' vs 'wall'
            obstacle = BlockObstacle(p0, p1) # Assumes BlockObstacle exists
            obstacles.append(obstacle)
            # print(f"Cluster {label}: Created BlockObstacle (len={length:.2f})")
        else:
            obstacle = WallObstacle(p0, p1) # Assumes WallObstacle exists
            obstacles.append(obstacle)
            # print(f"Cluster {label}: Created WallObstacle (len={length:.2f})")
            
    return obstacles