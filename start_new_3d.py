import numpy as np
from lib.simulation import FleetSimulation
from lib.robot import Fleet, si_to_uni, Robot
import control_algo
import control_algo_3d
from lib.mission import plot_mission_background
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# number of robots
# -----------------
nbRMEP= 1 # Number of robot master EP, cannon
nbTB3B = 3 # Number of robot TB3B, ground vehicle, burger
nbRMTT = 1 # Number of robot RMTT, drone, phantom 4
nbWaffle = 1 # Number of robot Waffle, unicycle type

# Deuxième flotte : DCA en singleIntegrator2D et un robot immobile
nbDCA_Fleet2 = 1 # Robot DCA pour la deuxième flotte
nbImmobile_Fleet2 = 1 # Robot immobile pour la deuxième flotte

# dynamics of robots
# -------------------
robotDynamics_EP = 'singleIntegrator3D'    # 3D intégrator for altitude
robotDynamics_TB3B = 'unicycle'           # use 'singleIntegrator2D' or 'unicycle'
robotDynamics_RMTT = 'singleIntegrator3D'  # 3D integrator for drones
robotDynamics_Waffle = 'unicycle'         # use 'singleIntegrator2D' or 'unicycle'
robotDynamics_Fleet2 = 'singleIntegrator3D'  # Deuxième flotte en singleIntegrator3D

# ... initial positions defined from data    (dimension: nb of agents  x  2)
# id of robots: 
id_fleet1 = ['DCA','burger1', 'burger2', 'burger3', 'waffle']
initPositions_fleet1 = np.array([[0, 0.4, 0.4, 0, 8],      # x-coordinates (m)
                                 [0, 0, -0.4,-0.4, 1.5],    # y-coordinates (m)
                                 [0, 0, 0, 0, 0]]).T       # z-coordinates (m) - all start at 0

# Positions pour la flotte des drones - adding altitude
initPositions_drone = np.array([[0], [-1], [1]])      # x,y,z-coordinates (m)

# Positions initiales pour la deuxième flotte - adding altitude
id_fleet2 = ['DCA_SI', 'Immobile']
initPositions_fleet2 = np.array([[0, 0],      # x-coordinates (m)
                                 [0, 0],      # y-coordinates (m) 
                                 [0, 0]]).T   # z-coordinates (m) - all start at 0

# Orientation angles pour la première flotte (unicycle robots)
initAngles_fleet1 = np.array([[0., 0., 0., 0., 0.]]).T
initPoses_fleet1 = np.concatenate((initPositions_fleet1[:, 0:2], initAngles_fleet1), axis=1)

# Nombre total de robots dans la simulation combinée
total_robots = nbRMEP + nbTB3B + nbWaffle + nbRMTT + nbDCA_Fleet2 + nbImmobile_Fleet2

# Création d'une flotte combinée manuellement avec tous les robots
combined_fleet = Fleet(0)  # Commencer avec une flotte vide

# Ajout des robots de la flotte 1 (DCA, burger1, burger2, burger3, waffle)
for i in range(nbRMEP + nbTB3B + nbWaffle):
    if i == 0:  # DCA is in 3D
        robot = Robot(robotDynamics_EP)
        robot.state = np.array([initPositions_fleet1[i, 0], initPositions_fleet1[i, 1], initPositions_fleet1[i, 2]])
    elif i == 4:  # waffle
        robot = Robot(robotDynamics_Waffle)
        robot.state = np.array([initPoses_fleet1[i, 0], initPoses_fleet1[i, 1], initPoses_fleet1[i, 2]])
    else:  # burgers
        robot = Robot(robotDynamics_TB3B)
        robot.state = np.array([initPoses_fleet1[i, 0], initPoses_fleet1[i, 1], initPoses_fleet1[i, 2]])
    
    combined_fleet.robot.append(robot)
    combined_fleet.nbOfRobots += 1

# Ajout des robots de la flotte RMTT (drones)
for i in range(nbRMTT):
    robot = Robot(robotDynamics_RMTT)
    # Positionner le drone RMTT au même endroit que le waffle
    waffle_position_x = initPositions_fleet1[4, 0]  # Position x du waffle (8)
    waffle_position_y = initPositions_fleet1[4, 1]  # Position y du waffle (1.5)
    waffle_position_z = 1.0  # Positionner le drone à 1 mètre au-dessus du waffle
    robot.state = np.array([waffle_position_x, waffle_position_y, waffle_position_z])
    combined_fleet.robot.append(robot)
    combined_fleet.nbOfRobots += 1

# Ajout des robots de la flotte 2 (DCA_SI et Immobile)
for i in range(nbDCA_Fleet2 + nbImmobile_Fleet2):
    robot = Robot(robotDynamics_Fleet2)
    # Ajuster les positions pour éviter les superpositions
    position_x = -1.5 if i == 0 else -2.0
    position_y = 1.0 if i == 0 else -1.0
    # DCA_SI (i==0) commence à 1m de hauteur, Immobile (i==1) à 0m
    position_z = 1.0 if i == 0 else 0.0
    robot.state = np.array([position_x, position_y, position_z])
    # --- Set robot number for DCA_SI and Immobile ---
    robot.robotNo = 6 + i
    combined_fleet.robot.append(robot)
    combined_fleet.nbOfRobots += 1

# Identification des robots dans la flotte combinée
combined_ids = id_fleet1 + ['RMTT'] + id_fleet2

# Print robot names
print("Robots in combined fleet:")
for i, robot_id in enumerate(combined_ids):
    print(f"Robot {i}: {robot_id}")

# sampling period for simulation
Ts = 0.01
# create simulation avec la flotte combinée
combined_simulation = FleetSimulation(combined_fleet, t0=0.0, tf=10.0, dt=Ts)

# Variables d'état pour la simulation
formation_finished = False
forward_finished = False
formation_bis_finished = False
dance_finished = False
oscillation_finished = False
pause_timer = 0
oscillation_start_time = 0

dance_started = False  # Flag to start dance phase after drone descent

# Mettre à jour le nombre de robots dans la flotte principale pour la fonction d'oscillation
import control_algo
control_algo.N_fleet1 = nbRMEP + nbTB3B + nbWaffle

# simulation loop for combined fleet
for t in combined_simulation.t:
    # get poses of robots as a single array of size (nb_of_robots, nb_of_states)
    combined_poses = combined_fleet.getPosesArray()
    
    N = combined_fleet.nbOfRobots
    
    # Tableau pour stocker les vitesses de tous les robots
    vx_all = np.zeros(N)
    vy_all = np.zeros(N)
    vz_all = np.zeros(N)  # Adding z-velocity for 3D support
    
    # === Traitement de la flotte principale (robots 0 à 4) ===
    N_fleet1 = nbRMEP + nbTB3B + nbWaffle
    fleet1_poses = combined_poses[:N_fleet1]
    
    # Calculer les vitesses pour les robots de la flotte principale avec support 3D
    if not forward_finished:
        vx_fleet1, vy_fleet1, vz_fleet1, forward_finished = control_algo_3d.forward_3d(
            t, N_fleet1, fleet1_poses, 
            np.array([[0.2,0] for _ in range(N_fleet1)]), 
            distance=2
        )
        vx_fleet1[4] = 0
        vy_fleet1[4] = 0
        vz_fleet1[4] = 0
        vx_drone = np.zeros(1)
        vy_drone = np.zeros(1)
        vz_drone = np.zeros(1)
        # --- DCA_SI (robot 6) follows DCA (robot 0) during forward phase ---
        if N > 6:
            dca_pos = combined_poses[0, :3]
            dca_si_pos = combined_poses[6, :3]
            kp_follow = 2.0
            error_follow = dca_pos - dca_si_pos
            vx_all[6] = kp_follow * error_follow[0]
            vy_all[6] = kp_follow * error_follow[1]
            vz_all[6] = kp_follow * error_follow[2]
    elif not formation_finished:
        r_ref_formation = np.zeros((N_fleet1, 2))
        r_ref_formation[1] = [-0.2, 0.4]
        r_ref_formation[2] = [0.4, -0.05]
        r_ref_formation[3] = [-0.2, -0.4]
        vx_fleet1, vy_fleet1, vz_fleet1, formation_finished = control_algo_3d.formation_3d(
            t, N_fleet1, fleet1_poses, r_ref_formation, 
            np.array([[0.2,0] for _ in range(N_fleet1)]), 
            distance=6
        )
        vx_drone = np.zeros(1)
        vy_drone = np.zeros(1)
        vz_drone = np.zeros(1)
        # --- DCA_SI (robot 6) follows DCA (robot 0) during formation phase ---
        if N >= 6:
            dca_pos = combined_poses[0, :3]
            dca_si_pos = combined_poses[6, :3]
            kp_follow = 2.0
            error_follow = dca_pos - dca_si_pos
            vx_all[6] = kp_follow * error_follow[0]
            vy_all[6] = kp_follow * error_follow[1]
            vz_all[6] = kp_follow * error_follow[2]
        
    elif not oscillation_finished:
        if pause_timer == 0:
            oscillation_start_time = t
            pause_timer = t
        # All fleet1 robots stop, only drone moves
        vx_fleet1 = np.zeros(N_fleet1)
        vy_fleet1 = np.zeros(N_fleet1)
        vz_fleet1 = np.zeros(N_fleet1)
        # Get velocities for ALL robots (including drone) from control_algo_3d
        vx_all, vy_all, vz_all, oscillation_finished = control_algo_3d.drone_oscillation_3d(
            t, N_fleet1 + nbRMTT, combined_poses, 
            np.array([[0,0] for _ in range(N)]),
            oscillation_start_time=oscillation_start_time
        )
        # Set drone velocities directly from vx_all, vy_all, vz_all
        vx_drone = np.array([vx_all[N_fleet1]])
        vy_drone = np.array([vy_all[N_fleet1]])
        vz_drone = np.array([vz_all[N_fleet1]])
        # --- NEW: Make the second drone (index N_fleet1+1) follow the DCA (index 0) ---
        if N > N_fleet1 + 1:
            dca_pos = combined_poses[0, :3]
            drone2_pos = combined_poses[N_fleet1+1, :3]
            kp_follow = 2.0
            error_follow = dca_pos - drone2_pos
            vx_all[N_fleet1+1] = kp_follow * error_follow[0]
            vy_all[N_fleet1+1] = kp_follow * error_follow[1]
            vz_all[N_fleet1+1] = kp_follow * error_follow[2]
        # Overwrite fleet1 velocities in vx_all, vy_all, vz_all for merging later
        vx_all[:N_fleet1] = vx_fleet1
        vy_all[:N_fleet1] = vy_fleet1
        vz_all[:N_fleet1] = vz_fleet1
        # Debug print
        print(f"Drone RMTT 3D velocity: {vx_drone}, {vy_drone}, {vz_drone}")
    else:
        if not dance_started:
            dance_started = True
            print("Starting dance phase for fleet1 around waffle robot!")
        # Fleet1 dances, drone and fleet2 stay still
        vx_fleet1, vy_fleet1, vz_fleet1, dance_finished = control_algo_3d.dance_3d(
            t, N_fleet1, fleet1_poses, 
            np.array([[0,0] for _ in range(N_fleet1)]),
            distance=2
        )
        vx_drone = np.zeros(1)
        vy_drone = np.zeros(1)
        vz_drone = np.zeros(1)
    
    # Z-velocity (altitude) is zero for all robots in the 2D plane
    vz_fleet1 = np.zeros(N_fleet1)
    
    # === Traitement de la flotte 2 (robots 6 et 7) ===
    # Index de début et fin des robots de la flotte 2 dans la flotte combinée
    start_idx_fleet2 = N_fleet1 + nbRMTT
    end_idx_fleet2 = N
    N_fleet2 = end_idx_fleet2 - start_idx_fleet2

    # Les robots DCA_SI et Immobile restent immobiles pendant toute la simulation
    vx_fleet2 = np.zeros(N_fleet2)
    vy_fleet2 = np.zeros(N_fleet2)
    vz_fleet2 = np.zeros(N_fleet2)

    # --- DCA_SI (robot 6, i=0 in fleet2) follows DCA (robot 0) during forward and formation phases ---
    if (not forward_finished or not formation_finished) and N >= 7:
        dca_pos = combined_poses[0, :3]
        dca_si_pos = combined_poses[6, :3]
        kp_follow = 2.0
        error_follow = dca_pos - dca_si_pos
        vx_fleet2[0] = kp_follow * error_follow[0]
        vy_fleet2[0] = kp_follow * error_follow[1]
        # Do not change vz for DCA_SI, keep it at zero
        vz_fleet2[0] = 0.0
    # Fusionner toutes les vitesses dans un seul tableau
    vx_all[:N_fleet1] = vx_fleet1
    vy_all[:N_fleet1] = vy_fleet1
    vz_all[:N_fleet1] = vz_fleet1

    vx_all[N_fleet1:N_fleet1+nbRMTT] = vx_drone
    vy_all[N_fleet1:N_fleet1+nbRMTT] = vy_drone
    vz_all[N_fleet1:N_fleet1+nbRMTT] = vz_drone

    vx_all[start_idx_fleet2:end_idx_fleet2] = vx_fleet2
    vy_all[start_idx_fleet2:end_idx_fleet2] = vy_fleet2
    vz_all[start_idx_fleet2:end_idx_fleet2] = vz_fleet2

    # Appliquer les contrôles à chaque robot
    for robotNo in range(combined_fleet.nbOfRobots):
        vxi, vyi, vzi = vx_all[robotNo], vy_all[robotNo], vz_all[robotNo]
        
        # Déterminer la dynamique du robot actuel
        current_robot = combined_fleet.robot[robotNo]

        # Robot waffle (index 4) reste fixe à (8, 1.5, 0)
        if robotNo == 4:  # waffle
            vx_all[robotNo] = 0.0
            vy_all[robotNo] = 0.0
            vz_all[robotNo] = 0.0
            vxi, vyi, vzi = 0.0, 0.0, 0.0
        # Robots de la flotte 2 (Immobile) restent immobiles (+1 car on veut DCA_SI mobile)
        elif robotNo >= start_idx_fleet2+1:
            vx_all[robotNo] = 0.0
            vy_all[robotNo] = 0.0
            vz_all[robotNo] = 0.0
            vxi, vyi, vzi = 0.0, 0.0, 0.0
        # Set the correct dynamics for each robot
        if robotNo < N_fleet1:
            if robotNo == 0:
                current_dynamics = 'singleIntegrator3D'
            elif robotNo == 4:
                current_dynamics = 'unicycle'
            else:
                current_dynamics = 'unicycle'
        elif robotNo < start_idx_fleet2:
            current_dynamics = 'singleIntegrator3D'
        else:
            current_dynamics = 'singleIntegrator3D'
        # Actually update the robot's control input
        if current_dynamics == 'singleIntegrator3D':
            current_robot.setCtrl(np.array([vxi, vyi, vzi]))
        elif current_dynamics == 'singleIntegrator2D':
            current_robot.setCtrl(np.array([vxi, vyi]))
        else:
            current_robot.setCtrl(si_to_uni(vxi, vyi, combined_poses[robotNo, 2], kp=15.))
    # Mettre à jour les données de simulation
    combined_simulation.addDataFromFleet(combined_fleet)
    # Intégrer le mouvement de la flotte
    combined_fleet.integrateMotion(Ts)

# Define custom 3D animation function
def animate_3d():
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Set the plot limits
    ax.set_xlim([-3, 10])
    ax.set_ylim([-3, 3])
    ax.set_zlim([0, 3])
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    
    # Colors for different robot types
    colors = ['red', 'green', 'blue', 'orange', 'purple', 'cyan', 'brown', 'pink']
    
    # Animation loop
    for i in range(0, len(combined_simulation.t), 5):  # Step every 5 frames for speed
        ax.clear()
        ax.set_xlim([-3, 10])
        ax.set_ylim([-3, 3])
        ax.set_zlim([0, 3])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        
        # Draw each robot
        for robot_idx in range(combined_fleet.nbOfRobots):
            # Get robot trajectory up to current time
            x = combined_simulation.robotSimulation[robot_idx].state[:i+1, 0]
            y = combined_simulation.robotSimulation[robot_idx].state[:i+1, 1]
            
            # Get z-coordinate or use 0 if 2D robot
            if combined_simulation.robotSimulation[robot_idx].robot.dynamics == 'singleIntegrator3D':
                z = combined_simulation.robotSimulation[robot_idx].state[:i+1, 2]
            else:
                z = np.zeros_like(x)
            
            # Current position
            current_x = x[-1]
            current_y = y[-1]
            current_z = z[-1]
            
            # Plot trajectory
            ax.plot(x, y, z, color=colors[robot_idx % len(colors)], alpha=0.5)
            
            # Plot current position (different marker for each robot type)
            if robot_idx == 0:  # DCA
                ax.scatter(current_x, current_y, current_z, color=colors[robot_idx % len(colors)], 
                          marker='s', s=100, label=f"{combined_ids[robot_idx]}")
            elif 1 <= robot_idx <= 3:  # Burgers
                ax.scatter(current_x, current_y, current_z, color=colors[robot_idx % len(colors)], 
                          marker='o', s=50, label=f"{combined_ids[robot_idx]}")
            elif robot_idx == 4:  # Waffle
                ax.scatter(current_x, current_y, current_z, color=colors[robot_idx % len(colors)], 
                          marker='*', s=150, label=f"{combined_ids[robot_idx]}")
            elif robot_idx == 5:  # Drone
                ax.scatter(current_x, current_y, current_z, color=colors[robot_idx % len(colors)], 
                          marker='^', s=100, label=f"{combined_ids[robot_idx]}")
            else:  # Fleet 2
                ax.scatter(current_x, current_y, current_z, color=colors[robot_idx % len(colors)], 
                          marker='d', s=80, label=f"{combined_ids[robot_idx]}")
        
        # Add legend (only on first frame)
        if i == 0:
            ax.legend(loc='upper right')
        
        # Title with current time
        ax.set_title(f"3D Robot Simulation - Time: {combined_simulation.t[i]:.2f}s")
        
        plt.pause(0.05)
    
    plt.show()

# Afficher l'animation 2D standard
combined_simulation.animation(figNo=1, pause=0.00001, robot_scale=0.2)

# Afficher l'animation 3D personnalisée
animate_3d()

# Afficher les trajectoires 2D
combined_simulation.plotXY(figNo=2)

# Afficher les états des robots au fil du temps
combined_simulation.plotState(figNo=3)

# Afficher les entrées de contrôle au fil du temps
combined_simulation.plotCtrl(figNo=4)

print("Simulation terminée avec les IDs de robots suivants:")
for i, robot_id in enumerate(combined_ids):
    print(f"Robot {i}: {robot_id}")
