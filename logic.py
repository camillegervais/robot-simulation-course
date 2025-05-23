import numpy as np
from lib.simulation import FleetSimulation
from lib.robot import Fleet, si_to_uni, Robot
import control_algo
from lib.mission import plot_mission_background 

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
robotDynamics_EP = 'singleIntegrator2D'    # use 'singleIntegrator2D' or 'unicycle' #De base controler en integrateur simple mais on veut un robot unicycle
robotDynamics_TB3B = 'unicycle'           # use 'singleIntegrator2D' or 'unicycle'
robotDynamics_RMTT = 'singleIntegrator2D'          # use 'singleIntegrator2D' or 'unicycle'
robotDynamics_Waffle = 'unicycle'         # use 'singleIntegrator2D' or 'unicycle'
robotDynamics_Fleet2 = 'singleIntegrator2D'  # Deuxième flotte en singleIntegrator2D

# ... initial positions defined from data    (dimension: nb of agents  x  2)
# id of robots: 
id_fleet1 = ['DCA','burger1', 'burger2', 'burger3', 'waffle']
initPositions_fleet1 = np.array([[0, 0.4, 0.4, 0, 8],      # x-coordinates (m)
                                 [0, 0, -0.4,-0.4, 1.5]]).T   # y-coordinates (m)

# Positions pour la flotte des drones
initPositions_drone = np.array([[0], [-1], [1]])      # x-coordinates (m)

# Positions initiales pour la deuxième flotte
id_fleet2 = ['DCA_SI', 'Immobile']
initPositions_fleet2 = np.array([[0, 0],      # x-coordinates (m)
                                 [0, 0]]).T   # y-coordinates (m)

# Orientation angles pour la première flotte (unicycle robots)
initAngles_fleet1 = np.array([[0., 0., 0., 0., 0.]]).T
initPoses_fleet1 = np.concatenate((initPositions_fleet1, initAngles_fleet1), axis=1)

# Nombre total de robots dans la simulation combinée
total_robots = nbRMEP + nbTB3B + nbWaffle + nbRMTT + nbDCA_Fleet2 + nbImmobile_Fleet2

# Création d'une flotte combinée manuellement avec tous les robots
combined_fleet = Fleet(0)  # Commencer avec une flotte vide

# Ajout des robots de la flotte 1 (DCA, burger1, burger2, burger3, waffle)
for i in range(nbRMEP + nbTB3B + nbWaffle):
    robot = Robot(robotDynamics_EP if i == 0 else (robotDynamics_Waffle if i == 4 else robotDynamics_TB3B))
    if robot.dynamics == 'unicycle':
        robot.state = np.array([initPoses_fleet1[i, 0], initPoses_fleet1[i, 1], initPoses_fleet1[i, 2]])
    else:
        robot.state = np.array([initPositions_fleet1[i, 0], initPositions_fleet1[i, 1]])
    combined_fleet.robot.append(robot)
    combined_fleet.nbOfRobots += 1

# Ajout des robots de la flotte RMTT (drones)
for i in range(nbRMTT):
    robot = Robot(robotDynamics_RMTT)
    # Ajuster les positions pour éviter les superpositions
    position_x = initPositions_drone[0] + 0.5
    position_y = initPositions_drone[i] if i < len(initPositions_drone) else 0
    robot.state = np.array([position_x, position_y])
    combined_fleet.robot.append(robot)
    combined_fleet.nbOfRobots += 1

# Ajout des robots de la flotte 2 (DCA_SI et Immobile)
for i in range(nbDCA_Fleet2 + nbImmobile_Fleet2):
    robot = Robot(robotDynamics_Fleet2)
    # Ajuster les positions pour éviter les superpositions
    position_x = -1.5 if i == 0 else -2.0
    position_y = 1.0 if i == 0 else -1.0
    robot.state = np.array([position_x, position_y])
    combined_fleet.robot.append(robot)
    combined_fleet.nbOfRobots += 1

# Identification des robots dans la flotte combinée
combined_ids = id_fleet1 + ['RMTT'] + id_fleet2

# sampling period for simulation
Ts = 0.01
# create simulation avec la flotte combinée
combined_simulation = FleetSimulation(combined_fleet, t0=0.0, tf=10.0, dt=Ts)

# Variables d'état pour la simulation
formation_finished = False
forward_finished = False
formation_bis_finished = False
dance_finished = False


def control(all_robots_poses, id):
    
    # get poses of robots as a single array of size (nb_of_robots, nb_of_states)    
    N = combined_fleet.nbOfRobots
    
    # Tableau pour stocker les vitesses de tous les robots
    vx_all = np.zeros(N)
    vy_all = np.zeros(N)
    
    # === Traitement de la flotte principale (robots 0 à 4) ===
    N_fleet1 = nbRMEP + nbTB3B + nbWaffle
    fleet1_poses = all_robots_poses[:N_fleet1]
    
    # Calculer les vitesses pour les robots de la flotte principale
    if not forward_finished:
        vx_fleet1, vy_fleet1, forward_finished = control_algo.forward(
            t, N_fleet1, fleet1_poses, 
            np.array([[0.2,0] for _ in range(N_fleet1)]), 
            distance=2
        )
    elif not formation_finished:
        r_ref_formation = np.zeros((N_fleet1, 2))
        r_ref_formation[1] = [0, 0.4]
        r_ref_formation[2] = [0.4, 0.2]
        r_ref_formation[3] = [0, -0.4]
        
        vx_fleet1, vy_fleet1, formation_finished = control_algo.formation(
            t, N_fleet1, fleet1_poses, r_ref_formation, 
            np.array([[0.2,0] for _ in range(N_fleet1)]), 
            distance=4
        )
    elif not formation_bis_finished:
        r_ref_formation_bis = np.zeros((N_fleet1, 2))
        r_ref_formation_bis[1] = [0, 0.4]
        r_ref_formation_bis[2] = [0.4, 0]
        r_ref_formation_bis[3] = [0, -0.4]
        
        vx_fleet1, vy_fleet1, formation_bis_finished = control_algo.formation(
            t, N_fleet1, fleet1_poses, r_ref_formation_bis, 
            np.array([[0.2,0] for _ in range(N_fleet1)]), 
            distance=6
        )
    elif not dance_finished:
        vx_fleet1, vy_fleet1, dance_finished = control_algo.dance(
            t, N_fleet1, fleet1_poses, 
            np.array([[0.2,0] for _ in range(N_fleet1)]), 
            distance=6
        )
    
    # === Traitement des drones (robot 5) ===
    # Les drones restent à leur position initiale
    vx_drone = np.zeros(nbRMTT)
    vy_drone = np.zeros(nbRMTT)
    
    # === Traitement de la flotte 2 (robots 6 et 7) ===
    # Index de début et fin des robots de la flotte 2 dans la flotte combinée
    start_idx_fleet2 = N_fleet1 + nbRMTT
    end_idx_fleet2 = N
    N_fleet2 = end_idx_fleet2 - start_idx_fleet2
    
    # Extraction des poses pour la flotte 2
    fleet2_poses = all_robots_poses[start_idx_fleet2:end_idx_fleet2]
    
    # Le robot DCA_SI (index 0 dans la flotte 2) suit le même comportement que le DCA de la flotte 1
    vx_fleet2 = np.zeros(N_fleet2)
    vy_fleet2 = np.zeros(N_fleet2)
    
    # Copier la vitesse du DCA original vers le DCA_SI
    if not forward_finished:
        vx_fleet2[0] = vx_fleet1[0]
        vy_fleet2[0] = vy_fleet1[0]
    elif not formation_finished or not formation_bis_finished:
        vx_fleet2[0] = vx_fleet1[0]
        vy_fleet2[0] = vy_fleet1[0]
    elif not dance_finished:
        vx_fleet2[0] = vx_fleet1[0]
        vy_fleet2[0] = vy_fleet1[0]
    
    # Le robot immobile (index 1) reste immobile
    vx_fleet2[1] = 0.0
    vy_fleet2[1] = 0.0
    
    # Fusionner toutes les vitesses dans un seul tableau
    vx_all[:N_fleet1] = vx_fleet1
    vy_all[:N_fleet1] = vy_fleet1
    
    vx_all[N_fleet1:N_fleet1+nbRMTT] = vx_drone
    vy_all[N_fleet1:N_fleet1+nbRMTT] = vy_drone
    
    vx_all[start_idx_fleet2:end_idx_fleet2] = vx_fleet2
    vy_all[start_idx_fleet2:end_idx_fleet2] = vy_fleet2
    
    # Appliquer les contrôles à chaque robot
    for robotNo in range(combined_fleet.nbOfRobots):
        vxi, vyi = vx_all[robotNo], vy_all[robotNo]
        
        # Déterminer la dynamique du robot actuel
        current_robot = combined_fleet.robot[robotNo]
        
        # Robot waffle (index 4) reste fixe à (8, 1.5)
        if robotNo == 4:  # waffle
            vx_all[robotNo] = 0.0
            vy_all[robotNo] = 0.0
            vxi, vyi = 0.0, 0.0
            
        if robotNo < N_fleet1:
            # Robots de la flotte 1 (DCA en singleIntegrator2D, les burgers en unicycle)
            if robotNo == 0:  # DCA
                current_dynamics = 'singleIntegrator2D'
            elif robotNo == 4:  # waffle
                current_dynamics = 'unicycle'
            else:  # burgers
                current_dynamics = 'unicycle'
        elif robotNo < start_idx_fleet2:  # Drones
            current_dynamics = 'singleIntegrator2D'
        else:  # Flotte 2 (singleIntegrator2D)
            current_dynamics = 'singleIntegrator2D'
        
        if current_dynamics == 'singleIntegrator2D':
            current_robot.ctrl = np.array([vxi, vyi])
        else:
            current_robot.ctrl = si_to_uni(vxi, vyi, all_robots_poses[robotNo, 2], kp=15.)
    
    return vx_all[id], vy_all[id]