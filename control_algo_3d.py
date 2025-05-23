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
    amplitude_x = 1.0
    frequency_x = 0.8
    relative_time = t - oscillation_start_time
    num_cycles = 2
    cycle_duration = 1.0 / frequency_x
    total_oscillation_time = num_cycles * cycle_duration
    oscillations_complete = relative_time >= total_oscillation_time
    for i in range(N):
        if i == rmtt_idx:
            drone_position = robots_poses[i, 0:3]
            if not oscillations_complete:
                # Oscillation ONLY on x, y fixed, descend in z
                target_x = waffle_position[0] + amplitude_x * np.sin(2 * np.pi * frequency_x * relative_time)
                target_y = waffle_position[1]
                z_start = waffle_position[2] + 1.0
                z_end = waffle_position[2] + 0.4
                total_time = total_oscillation_time
                target_z = z_start + (z_end - z_start) * min(relative_time / total_time, 1.0)
            else:
                target_x = waffle_position[0]
                target_y = waffle_position[1]
                target_z = waffle_position[2] + 1.0
            target_position = np.array([target_x, target_y, target_z])
            error = target_position - drone_position
            kp = 3.0
            vx[i] = kp * error[0]
            vy[i] = kp * error[1]
            vz[i] = kp * error[2]
            print(f"Drone target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}), error: ({error[0]:.2f}, {error[1]:.2f}, {error[2]:.2f})")
        else:
            vx[i] = 0.0
            vy[i] = 0.0
            vz[i] = 0.0
    finished = False
    if oscillations_complete:
        drone_position = robots_poses[rmtt_idx, 0:3]
        final_position = np.array([waffle_position[0], waffle_position[1], waffle_position[2] + 1.0])
        error_distance = np.linalg.norm(drone_position - final_position)
        if error_distance < 0.05:
            finished = True
            print("Drone oscillation phase complete - returning to normal operation")
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
