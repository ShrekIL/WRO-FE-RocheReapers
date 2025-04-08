from matplotlib import pyplot as plt
import numpy as np

from block_obstacle import BlockObstacle
from wall_obstacle import WallObstacle


class Trajectory:
    def __init__(self, current_pos, current_rotation, v, omega, time_horizon=1.5, sim_steps=20):
        """
        Generates a trajectory by simulating constant linear (v) and angular (omega)
        velocities from a starting pose over a time horizon.

        Args:
            current_pos (np.array): Starting position [x, y].
            current_rotation (float): Starting orientation (theta) in radians.
            v (float): Constant linear velocity for this trajectory.
            omega (float): Constant angular velocity for this trajectory.
            time_horizon (float): Duration of the simulation (seconds).
            sim_steps (int): Number of steps to discretize the trajectory simulation.
        """
        self.v = v
        self.omega = omega
        self.time_horizon = time_horizon
        self.sim_dt = time_horizon / sim_steps # time step for simulation

        self.path_poses = [] # List to store [(x, y, theta), ...] along the path
        x, y, theta = current_pos[0], current_pos[1], current_rotation
        self.path_poses.append(np.array([x, y, theta]))

        # Simulate the path using a simple kinematic model (adjust if needed, e.g., bicycle model)
        for _ in range(sim_steps):
            # Update theta first if using differential drive model update order
            # theta += self.omega * self.sim_dt # Option 1: Update theta before x,y
            # x += self.v * np.cos(theta) * self.sim_dt
            # y += self.v * np.sin(theta) * self.sim_dt

            # Option 2: Update using midpoint theta (often slightly more stable)
            x += self.v * np.cos(theta + 0.5 * self.omega * self.sim_dt) * self.sim_dt
            y += self.v * np.sin(theta + 0.5 * self.omega * self.sim_dt) * self.sim_dt
            theta += self.omega * self.sim_dt

            # Normalize theta to be within [-pi, pi] (optional but good practice)
            # theta = np.arctan2(np.sin(theta), np.cos(theta))

            self.path_poses.append(np.array([x, y, theta]))

        self.target_pos = self.path_poses[-1][:2]
        self.target_rotation = self.path_poses[-1][2]
        self.min_clearance = None

    def _get_min_obstacle_distance(self, obstacles, robot_radius):
        """
        Calculates the minimum clearance (distance - radius) from any point on
        the trajectory path to any obstacle in the list.
        **Assumes each obstacle object has a callable 'distance' attribute (method)**
        that takes a point [x, y] and returns the distance.

        Args:
            obstacles (list): List of obstacle objects, each expected to have a
                            `.distance(point)` method.
            robot_radius (float): The radius of the robot.

        Returns:
            float: The minimum clearance. Can be negative if a collision occurs.
                Returns float('inf') if obstacles list is empty or distance
                cannot be calculated for any obstacle using obs.distance().
        """
        # Initialize overall minimum CLEARANCE to infinity
        min_clearance_overall = float('inf')

        if not obstacles:
            # print("DEBUG: _get_min_obstacle_distance - No obstacles provided.")
            return min_clearance_overall # Return inf if no obstacles

        # Convert path poses to just XY points for easier distance calc
        path_xy = np.array([p[:2] for p in self.path_poses]) # Shape (N, 2)

        # Loop through each obstacle provided
        for i, obs in enumerate(obstacles):
            # Find the minimum squared distance from the path to THIS specific obstacle
            min_dist_sq_to_this_obs = float('inf')
            dist_calculated_for_obs = False # Flag to check if dist calc succeeded at least once

            # Check distance from all path points to this current obstacle
            for j, point_on_path in enumerate(path_xy):
                dist = float('nan') # Initialize dist for this point check

                try:
                    # --- Directly call obs.distance() ---
                    # This will raise AttributeError if 'distance' doesn't exist or isn't callable
                    dist = obs.distance(point_on_path)
                    # -------------------------------------

                    # Check if the returned distance is valid (not NaN or Inf)
                    if np.isnan(dist) or np.isinf(dist):
                        # Print warning only once per obstacle if distance calculation fails
                        if not dist_calculated_for_obs: # Check flag to print only once
                            print(f"DEBUG: obs.distance() returned invalid value ({dist}) for obs {i}, type {type(obs).__name__}")
                        dist = float('nan') # Ensure it's treated as invalid
                    else:
                        # Mark that at least one valid distance was calculated for this obs
                        dist_calculated_for_obs = True

                except AttributeError:
                    # Handle cases where obs does not have a 'distance' attribute/method
                    # Print only once per obstacle
                    if not hasattr(self, f'_warned_missing_distance_{i}'): # Use a temp attribute to track warning
                        print(f"DEBUG: Obstacle {i} type {type(obs).__name__} has no 'distance' attribute/method.")
                        setattr(self, f'_warned_missing_distance_{i}', True) # Mark as warned for this instance/obstacle index
                    # Since we must use .distance, we cannot proceed with this obstacle.
                    min_dist_sq_to_this_obs = float('inf') # Ensure this obs contributes inf distance
                    dist_calculated_for_obs = False
                    break # Stop checking points for this obstacle, move to next obstacle

                except Exception as e:
                    # Handle other unexpected errors during the obs.distance() call
                    if not hasattr(self, f'_warned_exception_distance_{i}'):
                        print(f"DEBUG: Error calling obs.distance() for obs {i} type {type(obs).__name__}: {e}")
                        setattr(self, f'_warned_exception_distance_{i}', True)
                    dist = float('nan') # Treat as invalid distance on error

                # --- Process the calculated distance ---
                # Check if distance is valid before squaring
                if np.isnan(dist):
                    continue # Skip this point if distance is not valid/calculable

                # If distance is valid, proceed
                dist_sq = dist * dist
                if dist_sq < min_dist_sq_to_this_obs:
                    min_dist_sq_to_this_obs = dist_sq
            # --- End of loop through path points for one obstacle ---

            # Clean up temporary warning flags for this trajectory instance
            if hasattr(self, f'_warned_missing_distance_{i}'): delattr(self, f'_warned_missing_distance_{i}')
            if hasattr(self, f'_warned_exception_distance_{i}'): delattr(self, f'_warned_exception_distance_{i}')

            # If a valid distance was calculated for this obstacle, update overall minimum
            if dist_calculated_for_obs and min_dist_sq_to_this_obs != float('inf'):
                actual_min_dist_to_obs = np.sqrt(min_dist_sq_to_this_obs)
                clearance_to_this_obs = actual_min_dist_to_obs - robot_radius

                # --- Update the OVERALL minimum clearance found so far ---
                if clearance_to_this_obs < min_clearance_overall:
                    min_clearance_overall = clearance_to_this_obs
                # print(f"DEBUG: Min clearance to obs {i} ({type(obs).__name__}) = {clearance_to_this_obs:.3f}. Overall = {min_clearance_overall:.3f}")
            # else:
                # print(f"DEBUG: No valid distance calculated for obs {i} ({type(obs).__name__}) or method missing/failed.")

        # --- End of loop through all obstacles ---

        # print(f"DEBUG: Final min_clearance_overall returned: {min_clearance_overall}")
        return min_clearance_overall


    def cost(self, goal, obstacles, robot_radius=0.15,
             goal_weight=1.0, obstacle_weight=2.0, heading_weight=0.5, velocity_weight=0.2, max_v=1.0):
        """
        Calculates the cost of the trajectory. Lower cost is better.

        Args:
            goal (np.array): Target goal position [x, y].
            obstacles (list): List of obstacle objects (e.g., ObstaclePoint).
            robot_radius (float): Radius of the robot for collision checking.
            goal_weight (float): Weight for the distance to goal component.
            obstacle_weight (float): Weight for the obstacle avoidance component.
            heading_weight (float): Weight for the heading towards goal component.
            velocity_weight (float): Weight for favouring forward velocity.
            max_v (float): Maximum possible linear velocity (for normalization).

        Returns:
            float: The calculated cost, or float('inf') if unsafe.
        """
        # 1. Obstacle Cost & Collision Check
        self.min_clearance = self._get_min_obstacle_distance(obstacles, robot_radius)

        if self.min_clearance < 0: # Collision detected
            return float("inf")

        # Use clearance. Higher clearance = lower cost. Avoid division by zero.
        # Cost increases sharply as clearance approaches 0.
        # Added 0.1 to prevent extreme values when clearance is tiny but positive.
        obstacle_cost = 1.0 / (self.min_clearance + 1e-6 + 0.1)

        # 2. Goal Distance Cost
        goal_dist = np.linalg.norm(self.target_pos - goal)
        goal_cost = goal_dist

        # 3. Goal Heading Cost
        goal_direction = np.arctan2(goal[1] - self.target_pos[1], goal[0] - self.target_pos[0])
        heading_diff = self.target_rotation - goal_direction
        # Normalize angle difference to [-pi, pi]
        heading_diff = np.arctan2(np.sin(heading_diff), np.cos(heading_diff))
        heading_cost = abs(heading_diff)

        # 4. Velocity Cost (Optional: Encourage higher forward speed)
        # Lower cost for higher v (closer to max_v)
        velocity_cost = (max_v - self.v) / max_v if max_v > 0 else 0


        # 5. Combine Costs (Normalization might be needed depending on ranges)
        # Simple weighted sum - requires tuning weights carefully!
        total_cost = (obstacle_weight * obstacle_cost +
                      goal_weight * goal_cost +
                      heading_weight * heading_cost +
                      velocity_weight * velocity_cost)

        return total_cost

    def plot(self, color='b--', label_prefix="Trajectory"):
        """Plots the trajectory path."""
        path_xy = np.array([p[:2] for p in self.path_poses])
        plt.plot(path_xy[:, 0], path_xy[:, 1], color, label=f"{label_prefix} (v={self.v:.2f}, w={self.omega:.2f})")
        # Plot endpoint
        plt.plot(self.target_pos[0], self.target_pos[1], 'go') # Green circle for endpoint
