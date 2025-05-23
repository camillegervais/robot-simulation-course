import numpy as np

# Variable pour vérifier le premier appel de chaque fonction
global firstCall_oscillation
firstCall_oscillation = True

# Variable to store the number of robots in the main fleet (needed for drone oscillation)
global N_fleet1
N_fleet1 = 5  # Default value, will be updated in the start.py file

# Function to calculate drone oscillation in 3D 
def drone_oscillation_3d(t, robotNo, robots_poses, robots_velocities, oscillation_start_time=0, distance=1.5):
    global firstCall_oscillation
    global N_fleet1
    N = robots_poses.shape[0]
    vx = np.zeros(N)
    vy = np.zeros(N)
    vz = np.zeros(N)
    if (firstCall_oscillation):
        print("Starting drone oscillation phase above waffle robot in 3D")
        print(f"Total robots: {N}, Fleet1 robots: {N_fleet1}, Drone idx: {N_fleet1}")
        firstCall_oscillation = False
    waffle_idx = N_fleet1 - 1
    waffle_position = robots_poses[waffle_idx, 0:3]
    rmtt_idx = N_fleet1
    drone_position = robots_poses[rmtt_idx, 0:3]
    # Make the drone oscillate in x while going down in z (like hurt)
    amplitude_x = 0.7
    frequency_x = 1.2  # Hz
    relative_time = t - oscillation_start_time
    target_x = waffle_position[0] + amplitude_x * np.sin(2 * np.pi * frequency_x * relative_time)
    target_y = waffle_position[1]
    # Descend in z, but not straight: slow descent
    z_start = waffle_position[2] + 1.0
    z_end = 0.25
    descent_duration = 3.0  # seconds
    frac = min(relative_time / descent_duration, 1.0)
    target_z = z_start + (z_end - z_start) * frac
    target_position = np.array([target_x, target_y, target_z])
    error = target_position - drone_position
    kp_x = 6.0
    kp_y = 6.0
    kp_z = 3.0
    vx[rmtt_idx] = kp_x * error[0]
    vy[rmtt_idx] = kp_y * error[1]
    vz[rmtt_idx] = kp_z * error[2]
    # All other robots remain stationary
    for i in range(N):
        if i != rmtt_idx:
            vx[i] = 0.0
            vy[i] = 0.0
            vz[i] = 0.0
    # Finish when drone is close to ground
    finished = False
    if abs(drone_position[2] - z_end) < 0.05:
        finished = True
        print("Drone has finished its hurt descent.")
    return vx, vy, vz, finished

# Pass-through functions for compatibility with original control_algo
def forward_3d(t, robotNo, robots_poses, robots_velocities, distance=1.5):
    # Just call the original forward function but return zeros for z-velocity
    from control_algo import forward
    vx, vy, finished = forward(t, robotNo, robots_poses, robots_velocities, distance)
    vz = np.zeros_like(vx)
    return vx, vy, vz, finished

def formation_3d(t, robotNo, robots_poses, r_ref, robots_velocities, distance=1.5):
    # Just call the original formation function but return zeros for z-velocity
    from control_algo import formation
    vx, vy, finished = formation(t, robotNo, robots_poses, r_ref, robots_velocities, distance)
    vz = np.zeros_like(vx)
    return vx, vy, vz, finished

def dance_3d(t, robotNo, robots_poses, robots_velocities, distance=1.5):
    # Just call the original dance function but return zeros for z-velocity
    from control_algo import dance
    vx, vy, finished = dance(t, robotNo, robots_poses, robots_velocities, distance)
    vz = np.zeros_like(vx)
    return vx, vy, vz, finished
