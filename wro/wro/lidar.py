import sys
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
import time

from wall_obstacle import WallObstacle # Example placeholder
from block_obstacle import BlockObstacle # Example placeholder

# Assume this module contains the DWA-style generate_trajectories and Trajectory class
from path_planing import generate_trajectories, Trajectory # Make sure Trajectory class is also imported or defined

# --- Robot Parameters (CRUCIAL - ADJUST THESE!) ---
V_MIN = 0.0       # Minimum linear velocity (m/s)
V_MAX = 0.5       # Maximum linear velocity (m/s) - Start lower
OMEGA_MIN = -np.deg2rad(60) # Minimum angular velocity (rad/s)
OMEGA_MAX = np.deg2rad(60)  # Maximum angular velocity (rad/s)
A_LIN_MAX = 0.5   # Max linear acceleration/deceleration (m/s^2)
A_ANG_MAX = np.deg2rad(90) # Max angular acceleration/deceleration (rad/s^2)

CONTROL_DT = 0.1         # Control loop time step (s) - Use this for simulation step!
TRAJ_TIME_HORIZON = 2.5  # Trajectory simulation time (s)
TRAJ_SIM_STEPS = 20      # Trajectory simulation steps

V_SAMPLES = 5            # Number of V samples in DW
OMEGA_SAMPLES = 31       # Number of Omega samples in DW

ROBOT_RADIUS = 0.10      # Robot radius for collision checking (m) - IMPORTANT!

# --- Cost Function Weights (CRUCIAL - TUNE THESE!) ---
GOAL_WEIGHT = 12.0
OBSTACLE_WEIGHT = 2.5    # Increase this significantly!
HEADING_WEIGHT = 2.8
VELOCITY_WEIGHT = 8    # Lower penalty for non-max velocity

MAX_V_FOR_COST = V_MAX # Used in velocity cost normalization

# --- Lidar Processing ---
try:
    scan = np.load('/home/kuttelr/liadar.npy')
except FileNotFoundError:
    print("Error: Lidar scan file not found. Exiting.")
    sys.exit(1)

# Step 1: Cluster points using DBSCAN
start = time.time()
# Consider adjusting eps based on lidar density/expected object separation
dbscan = DBSCAN(eps=0.2, min_samples=3) # Slightly higher min_samples might reduce noise clusters
labels = dbscan.fit_predict(scan)
end = time.time()
print(f"DBSCAN took {end - start:.4f} seconds")

unique_labels = set(labels)
print(f"Found {len(unique_labels)- (1 if -1 in unique_labels else 0)} clusters (excluding noise).")

# Step 2: Fit RANSAC for each cluster and create obstacles
lines = []
obstacles = []

for label in unique_labels:
    if label == -1:  # Ignore noise points labeled as -1
        plt.scatter(scan[labels == label, 0], scan[labels == label, 1], c='gray', s=5, label='Noise')
        continue

    cluster_points = scan[labels == label]

    # Ensure enough points for RANSAC (needs at least 2)
    if len(cluster_points) < 2:
        print(f"Skipping cluster {label}: Not enough points ({len(cluster_points)}).")
        continue

    X_cluster = cluster_points[:, 0].reshape(-1, 1)
    Y_cluster = cluster_points[:, 1]

    try:
        # Fit RANSAC to the cluster
        start_r = time.time()
        ransac = RANSACRegressor(residual_threshold=0.05) # Add a residual threshold
        ransac.fit(X_cluster, Y_cluster)
        end_r = time.time()
        #print(f"RANSAC took {end_r - start_r:.4f} seconds for cluster {label}")

        # Identify inlier points according to RANSAC
        inlier_mask = ransac.inlier_mask_
        inlier_points = cluster_points[inlier_mask]

        # If very few inliers, might be noise or non-linear shape
        if len(inlier_points) < 3:
             print(f"Skipping cluster {label}: Too few RANSAC inliers ({len(inlier_points)}).")
             plt.scatter(X_cluster[:,0], Y_cluster, marker='x', color='orange', label=f'Cluster {label} (Skipped)')
             continue

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
        if length < 0.05: # Increase threshold slightly?
            print(f"Skipping cluster {label}: RANSAC result too short ({length:.3f}m).")
            plt.scatter(X_cluster[:,0], Y_cluster, marker='x', color='pink', label=f'Cluster {label} (Too short)')
            continue

        # Create Obstacle based on length (use your specific classes)
        if length < 0.15: # Example threshold for 'block' vs 'wall'
            obstacle = BlockObstacle(p0, p1) # Assumes BlockObstacle exists
            obstacles.append(obstacle)
            # print(f"Cluster {label}: Created BlockObstacle (len={length:.2f})")
        else:
            obstacle = WallObstacle(p0, p1) # Assumes WallObstacle exists
            obstacles.append(obstacle)
            # print(f"Cluster {label}: Created WallObstacle (len={length:.2f})")

        # Plotting for verification
        plt.scatter(inlier_points[:, 0], inlier_points[:, 1], label=f"Cluster {label} Inliers")
        # Plot outliers if needed:
        # outlier_points = cluster_points[~inlier_mask]
        # plt.scatter(outlier_points[:, 0], outlier_points[:, 1], marker='x', color='grey')


    except ValueError as e:
        print(f"RANSAC failed for cluster {label}: {e}")
        plt.scatter(X_cluster[:,0], Y_cluster, marker='x', color='red', label=f'Cluster {label} (RANSAC Fail)')

print(f"Created {len(obstacles)} obstacles from Lidar scan.")

obstacles2 = [    
    WallObstacle(start=[-1.5, 1.5], end=[1.5, 1.5]), # top
    WallObstacle(start=[-1.5, -1.5], end=[1.5, -1.5]),  # bottom
    WallObstacle(start=[-1.5, 1.5], end=[-1.5, -1.5]),  # left
    WallObstacle(start=[1.5, 1.5], end=[1.5, -1.5]),  # right

    WallObstacle(start=[-0.75, 0.75], end=[0.75, 0.75]), # top
    WallObstacle(start=[-0.75, -0.75], end=[0.75, -0.75]),  # bottom
    WallObstacle(start=[-0.75, 0.75], end=[-0.75, -0.75]),  # left
    WallObstacle(start=[0.75, 0.75], end=[0.75, -0.75]),  # right
    
    BlockObstacle(pos=[0, 1.125]),
]
goal = np.array([0,1.125])  # Example goal


# --- Simulation Loop ---
position = np.array([-1.125, 0]) # Ensure float
rotation = np.deg2rad(90)    # Start facing positive Y example (adjust)
current_v = 0.0            # Start at rest
current_omega = 0.0        # Start at rest

plt.figure() # Create figure for animation

iteration = 0
max_iterations = 200

while np.linalg.norm(position - goal) > 0.15 and iteration < max_iterations: # Added iteration limit
    plt.cla() # Clear axes for animation

    # Plot Goal and Robot
    plt.scatter(goal[0], goal[1], marker='*', s=200, c="orange", label="Goal", zorder=5)
    plt.plot(position[0], position[1], 'o', markersize=8, c="purple", label="Robot", zorder=5)
    # Draw arrow for robot orientation
    arrow_len = 0.3
    plt.arrow(position[0], position[1],
              arrow_len * np.cos(rotation), arrow_len * np.sin(rotation),
              head_width=0.08, head_length=0.15, fc='purple', ec='purple', zorder=5)


    # Plot Obstacles
    for o in obstacles:
        o.plot() # Assumes obstacles have a plot method

    # --- DWA Core ---
    # 1. Generate Trajectories using current state and limits
    trajectories = generate_trajectories(
        current_pos=position,
        current_rot=rotation,
        current_v=current_v,
        current_omega=current_omega, # Use current angular velocity
        v_min=V_MIN, v_max=V_MAX,
        omega_min=OMEGA_MIN, omega_max=OMEGA_MAX,
        a_lin_max=A_LIN_MAX, a_ang_max=A_ANG_MAX,
        control_dt=CONTROL_DT,             # Pass control_dt for DW calculation
        traj_time_horizon=TRAJ_TIME_HORIZON,
        traj_sim_steps=TRAJ_SIM_STEPS,
        v_samples=V_SAMPLES,
        omega_samples=OMEGA_SAMPLES
    )

    if not trajectories:
        print("Warning: No valid trajectories generated. Stopping.")
        break # Exit if planning fails

    # 2. Evaluate Trajectories
    costs = []
    valid_trajectories = []
    for t in trajectories:
        # Calculate cost using weights and robot radius defined above
        cost = t.cost(
            goal=goal,
            obstacles=obstacles,
            robot_radius=ROBOT_RADIUS,
            goal_weight=GOAL_WEIGHT,
            obstacle_weight=OBSTACLE_WEIGHT,
            heading_weight=HEADING_WEIGHT,
            velocity_weight=VELOCITY_WEIGHT,
            max_v = MAX_V_FOR_COST
        )
        # Store valid trajectories and their costs
        if cost != float('inf'):
             costs.append(cost)
             valid_trajectories.append(t)
        # else:
             # Optionally plot colliding trajectories differently
             # t.plot(color='r:') # Plot colliding in red dotted

    if not valid_trajectories:
        print("Warning: All generated trajectories lead to collision or are invalid! Stopping.")
        # Optionally plot all generated trajectories before stopping
        for t in trajectories:
            t.plot(color='m:') # Magenta dotted for all attempted paths
        plt.pause(2.0)
        break

    # Find the best valid trajectory
    best_traj_index = np.argmin(costs)
    best_trajectory = valid_trajectories[best_traj_index]
    
    best_traj_clearance = best_trajectory.min_clearance
    # Actual distance = clearance + radius. Handle potential None if cost wasn't called (shouldn't happen here)
    min_dist_to_obstacle = (best_traj_clearance + ROBOT_RADIUS) if best_traj_clearance is not None else float('nan')
    print(f"Iter {iteration}: Best Traj Cost={costs[best_traj_index]:.2f}. Min Obstacle Distance = {min_dist_to_obstacle:.3f} m (Clearance = {best_traj_clearance:.3f} m)")

    # --- Plotting (Optional: plot all valid trajectories) ---
    # for i, t in enumerate(valid_trajectories):
    #     if i == best_traj_index:
    #         t.plot(color='g-', label_prefix="*** Best") # Best in solid green
    #     else:
    #         t.plot(color='b:') # Others in blue dotted
    # Plot only the best one for clarity
    best_trajectory.plot(color='g-', label_prefix="Best")


    # 3. Get Commands from Best Trajectory
    chosen_v = best_trajectory.v
    chosen_omega = best_trajectory.omega

    # --- Update Robot State (FIXED: Simulate one step, don't teleport) ---
    # Use the actual CONTROL_DT for simulation step
    dt = CONTROL_DT
    # Kinematic update (using midpoint velocity for slightly better accuracy)
    delta_x = chosen_v * np.cos(rotation + 0.5 * chosen_omega * dt) * dt
    delta_y = chosen_v * np.sin(rotation + 0.5 * chosen_omega * dt) * dt
    delta_rot = chosen_omega * dt

    position = position + np.array([delta_x, delta_y])
    rotation = rotation + delta_rot
    # Normalize rotation angle to [-pi, pi]
    rotation = np.arctan2(np.sin(rotation), np.cos(rotation))

    # Update current velocities for the next planning cycle
    current_v = chosen_v
    current_omega = chosen_omega # Use consistent variable name

    # --- Plotting and Pause ---
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.axis("equal")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    # Dynamically adjust limits or set fixed ones
    plt.grid(True)
    plt.pause(0.1) # Adjust pause duration

    iteration += 1

# --- End of Loop ---
if iteration == max_iterations:
    print("Max iterations reached.")
elif np.linalg.norm(position - goal) <= 0.15:
    print("Goal reached!")
else:
    print("Loop exited for other reasons.")

plt.show() # Keep final plot open