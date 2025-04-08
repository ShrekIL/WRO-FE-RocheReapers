import numpy as np
from trajectory import Trajectory
from typing import List

max_rotation = 1
max_d_rotation_per_s = 1 # rad / s
max_d_d_rotation_per_s = 8 # rad / s^2
max_d_rotation = 1 # rad / s

speed = 1

resolution = 1 / 15

def generate_trajectories(
    current_pos: np.ndarray,
    current_rot: float,
    current_v: float,
    current_omega: float,
    v_min: float = -1,
    v_max: float = 1,
    omega_min: float = -1, # rad / s
    omega_max: float = 1, # rad / s
    a_lin_max: float = 1.5,  # Max linear acceleration/deceleration
    a_ang_max: float = 2.5,  # Max angular acceleration/deceleration
    control_dt: float = 0.1, # Time step over which acceleration limits apply (Dynamic Window calc)
    traj_time_horizon: float = 0.1, # Simulation time for each trajectory
    traj_sim_steps: int = 20,      # Simulation steps for each trajectory
    v_samples: int = 7,           # Number of linear velocity samples
    omega_samples: int = 5        # Number of angular velocity samples
    ) -> list[Trajectory]:
    """
    Generates a list of possible trajectories within the dynamic window.

    Args:
        current_pos: Robot's current position [x, y].
        current_rot: Robot's current orientation (theta) in radians.
        current_v: Robot's current linear velocity.
        current_omega: Robot's current angular velocity.
        v_min: Minimum possible linear velocity.
        v_max: Maximum possible linear velocity.
        omega_min: Minimum possible angular velocity (can be negative).
        omega_max: Maximum possible angular velocity.
        a_lin_max: Maximum linear acceleration AND deceleration.
        a_ang_max: Maximum angular acceleration AND deceleration.
        control_dt: Time step (in seconds) for calculating the dynamic window.
        traj_time_horizon: Simulation time (in seconds) for generating each trajectory.
        traj_sim_steps: Number of simulation steps for trajectory generation.
        v_samples: Number of samples to take for linear velocity.
        omega_samples: Number of samples to take for angular velocity.

    Returns:
        A list of generated Trajectory objects.
    """

    # 1. Calculate the Dynamic Window (allowable velocities within control_dt)
    v_lower_dw = max(v_min, current_v - a_lin_max * control_dt)
    v_upper_dw = min(v_max, current_v + a_lin_max * control_dt)

    omega_lower_dw = max(omega_min, current_omega - a_ang_max * control_dt)
    omega_upper_dw = min(omega_max, current_omega + a_ang_max * control_dt)

    possible_trajectories = []

    # Ensure there's a range to sample from
    if v_upper_dw < v_lower_dw:
         # This might happen if deceleration is needed but v_min limits it
         # A simple fix is to allow staying at v_min if accelerating down would cross it
         # Or handle based on specific robot behavior (e.g., force stop)
         # For now, we might just allow the lower bound if upper is less
         v_upper_dw = v_lower_dw # Or perhaps better: just use current_v if it's within bounds? Adjust logic as needed.
         print(f"Warning: Velocity dynamic window inverted ({v_lower_dw:.2f} > {v_upper_dw:.2f}). Clamping.")
         if v_lower_dw > v_max: v_lower_dw = v_max # Further clamp if needed
         if v_upper_dw < v_min: v_upper_dw = v_min

    if omega_upper_dw < omega_lower_dw:
         omega_upper_dw = omega_lower_dw # Similar clamping logic for omega
         print(f"Warning: Omega dynamic window inverted ({omega_lower_dw:.2f} > {omega_upper_dw:.2f}). Clamping.")
         if omega_lower_dw > omega_max: omega_lower_dw = omega_max
         if omega_upper_dw < omega_min: omega_upper_dw = omega_min


    # 2. Sample velocities within the dynamic window
    # Use linspace to ensure bounds are included and sampling is even
    v_range = np.linspace(v_lower_dw, v_upper_dw, v_samples) if v_samples > 1 else [v_upper_dw] # Handle edge case of 1 sample
    omega_range = np.linspace(omega_lower_dw, omega_upper_dw, omega_samples) if omega_samples > 1 else [omega_upper_dw]

    # 3. Create trajectories for each sampled (v, omega) pair
    for v_sample in v_range:
        for omega_sample in omega_range:
            # Basic check: Don't generate trajectories that only rotate in place if v=0 unless omega is also 0
            # (avoids unnecessary computation, can be adjusted based on need)
            if abs(v_sample) < 1e-4 and abs(omega_sample) < 1e-4 and (current_v > 1e-4 or current_omega > 1e-4):
                 # Allow (0,0) only if already stopped or if it's the only option
                 if v_lower_dw <= 0 <= v_upper_dw and omega_lower_dw <=0 <= omega_upper_dw:
                     pass # Allow the stop command (0,0)
                 else:
                    continue # Skip purely rotational trajectories if moving is possible

            # Create the trajectory object by simulating forward
            traj = Trajectory(
                current_pos=current_pos,
                current_rotation=current_rot,
                v=v_sample,
                omega=omega_sample,
                time_horizon=traj_time_horizon,
                sim_steps=traj_sim_steps
            )
            possible_trajectories.append(traj)

    # Always ensure the "stop" trajectory (0, 0) is considered if physically possible
    # This is crucial for safety and reaching goals precisely.
    is_zero_vel_possible = (v_lower_dw <= 0.0 <= v_upper_dw) and (omega_lower_dw <= 0.0 <= omega_upper_dw)
    has_zero_vel_traj = any(abs(t.v) < 1e-4 and abs(t.omega) < 1e-4 for t in possible_trajectories)

    if is_zero_vel_possible and not has_zero_vel_traj:
         print("Info: Adding explicit (0,0) trajectory.")
         stop_traj = Trajectory(
                current_pos=current_pos,
                current_rotation=current_rot,
                v=0.0,
                omega=0.0,
                time_horizon=traj_time_horizon,
                sim_steps=traj_sim_steps
            )
         possible_trajectories.append(stop_traj)


    print(f"Generated {len(possible_trajectories)} trajectories from v:[{v_lower_dw:.2f}, {v_upper_dw:.2f}], w:[{omega_lower_dw:.2f}, {omega_upper_dw:.2f}]")
    return possible_trajectories
