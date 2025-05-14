import numpy as np
from lib.simulation import FleetSimulation
from lib.robot import Fleet, si_to_uni
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
robotDynamics_EP = 'unicycle'    # use 'singleIntegrator2D' or 'unicycle' #De base controler en integrateur simple mais on veut un robot unicycle
robotDynamics_TB3B = 'unicycle'           # use 'singleIntegrator2D' or 'unicycle'
robotDynamics_RMTT = 'singleIntegrator2D'          # use 'singleIntegrator2D' or 'unicycle'
robotDynamics_Waffle = 'unicycle'         # use 'singleIntegrator2D' or 'unicycle'
robotDynamics_Fleet2 = 'singleIntegrator2D'  # Deuxième flotte en singleIntegrator2D

robotDynamics = 'unicycle' # use 'signleIntegrator2D' or 'unicycle'
# ... initial positions randomly defined 
#initPositions = 20*np.random.rand(nbOfRobots,2)-10  # random init btw -10, +10
# ... initial positions defined from data    (dimension: nb of agents  x  2)
# id of robots: 
id = ['DCA','burger1', 'burger2', 'burger3', 'waffle']
initPositions = np.array([[0, 0.4, 0.4, 0, 8],      # x-coordinates (m)
                          [0, 0, -0.4,-0.4, 1.5]]).T   # y-coordinates (m)
initPositions_general_burger = np.array([[0], [-1], [1]])      # x-coordinates (m)

# Positions initiales pour la deuxième flotte
id_fleet2 = ['DCA_SI', 'Immobile']
initPositions_fleet2 = np.array([[0, 0],      # x-coordinates (m)
                                 [0, 0]]).T   # y-coordinates (m)

# Formation of the fleet, 40 cm de distance entre les centres des robots, de haut en bas burger 1, 2, 3 

if (robotDynamics=='unicycle'):
    # orientation angles (rad)    (dimension: nb of agnts x 1)
    initAngles = np.array([[0., 0., 0., 0., 0.]]).T
    initPoses = np.concatenate((initPositions, initAngles), axis=1)


# create fleet for formation control
if (robotDynamics=='singleIntegrator2D'):
    fleet = Fleet(nbRMEP + nbTB3B + nbWaffle, dynamics=robotDynamics, initStates=initPositions)
else:
    fleet = Fleet(nbRMEP + nbTB3B + nbWaffle, dynamics=robotDynamics, initStates=initPoses)

# create fleet for drone control
fleet_RMTT = Fleet(nbRMTT, dynamics=robotDynamics_RMTT, initStates=initPositions_general_burger)

# Création de la deuxième flotte avec DCA et robot immobile
fleet2 = Fleet(nbDCA_Fleet2 + nbImmobile_Fleet2, dynamics=robotDynamics_Fleet2, initStates=initPositions_fleet2)


# sampling period for simulation
Ts = 0.01
# create simulation
simulation = FleetSimulation(fleet, t0=0.0, tf=10.0, dt=Ts)
simulation_drone = FleetSimulation(fleet_RMTT, t0=0.0, tf=10.0, dt=Ts)
simulation_fleet2 = FleetSimulation(fleet2, t0=0.0, tf=10.0, dt=Ts)  # Simulation de la deuxième flotte

formation_finished = False
forward_finished = False
formation_bis_finished = False
dance_finished = False

# Variables pour la deuxième flotte
formation_finished_fleet2 = False
forward_finished_fleet2 = False
formation_bis_finished_fleet2 = False
dance_finished_fleet2 = False

# simulation loop


for t in simulation.t:
    # get poses of robots as a single array of size (nb_of_robots, nb_of_states)
    robots_poses = fleet.getPosesArray()
    
    N = fleet.nbOfRobots
    # print("vx : ",vx)
    # print("vy : ",vy)
    # vx= [0,0,0,0]
    # vy = [0.5,0.5,0.5,0.5]
    if not forward_finished:
        vx ,vy, forward_finished = control_algo.forward(t, N, robots_poses, np.array([[0.2,0],[0.,0],[0.,0],[0.,0],[0.,0]]), distance=2)  # <= MODIFY CONTROL LAW IN "control_algo.py"
        
    elif not formation_finished:
        r_ref_formation = np.array([[0,0],[0,0.4],[0.4,0.2],[0,-0.4],[0,0]])  # Ajout d'une référence pour waffle
        vx, vy, formation_finished = control_algo.formation(t, N, robots_poses, r_ref_formation, np.array([[0.2,0],[0.,0],[0.,0],[0.,0],[0.,0]]), distance=4)  # <= MODIFY CONTROL LAW IN "control_algo.py"

    elif not formation_bis_finished:
        r_ref_formation_bis = np.array([[0,0],[0,0.4],[0.4,0],[0,-0.4],[0,0]])  # Ajout d'une référence pour waffle
        vx, vy, formation_bis_finished = control_algo.formation(t, N, robots_poses, r_ref_formation_bis, np.array([[0.2,0],[0.,0],[0.,0],[0.,0],[0.,0]]), distance=6)

    elif not dance_finished:
        vx, vy, dance_finished = control_algo.dance(t, N, robots_poses, np.array([[0.2,0],[0.,0],[0.,0],[0.,0],[0.,0]]), distance=6)
    
        
    # elif formation_finished:
    #     print("Formation finished")   
    #     vx ,vy, forward_finished = control_algo.forward(t, N, robots_poses, np.array([[0,0],[0.,0],[0.,0],[0.,0]])   )  # <= MODIFY CONTROL LAW IN "control_algo.py"

    for robotNo in range(fleet.nbOfRobots):
        vxi,vyi = vx[robotNo], vy[robotNo]
        if (robotDynamics=='singleIntegrator2D'):
            fleet.robot[robotNo].ctrl = np.array([vxi, vyi]) 
        else:
            fleet.robot[robotNo].ctrl = si_to_uni(vxi, vyi, robots_poses[robotNo,2], kp=15.) 
    
    # update data of simulation 
    simulation.addDataFromFleet(fleet)
    # integrate motion of the fleet
    fleet.integrateMotion(Ts)

# for t in simulation_drone.t:
#     # get poses of robots as a single array of size (nb_of_robots, nb_of_states)
#     robots_poses = fleet_RMTT.getPosesArray()
#     print("flett_RMTT _ state dim : ",fleet_RMTT.robot[0].stateDim)
#     print("flett_RMTT _ state  : ",fleet_RMTT.robot[0].state)

#     print("robots_poses : ", robots_poses)
    
#     # print("vx : ",vx)
#     # print("vy : ",vy)
#     vx ,vy, forward_finished = control_algo.forward(t, 1, robots_poses, np.array([[0.2,0]]), distance=2   )  # <= MODIFY CONTROL LAW IN "control_algo.py"

#     fleet_RMTT.robot[0].ctrl = np.array([vx, vy]) 

    
#     # update data of simulation 
#     simulation_drone.addDataFromFleet(fleet_RMTT)
#     # integrate motion of the fleet
#     fleet_RMTT.integrateMotion(Ts)

# simulation de la deuxième flotte avec DCA_SI et le robot immobile
for t in simulation_fleet2.t:
    # get poses of robots as a single array of size (nb_of_robots, nb_of_states)
    robots_poses_fleet2 = fleet2.getPosesArray()
    robots_poses = fleet.getPosesArray()  # Pour récupérer la position du DCA original
    
    N_fleet2 = fleet2.nbOfRobots
    
    # Initialiser les vitesses pour la flotte 2
    vx_fleet2 = np.zeros(N_fleet2)
    vy_fleet2 = np.zeros(N_fleet2)
    
    # Pour le robot DCA_SI (index 0), appliquer le même comportement que DCA de la flotte 1
    if not forward_finished_fleet2:
        # Obtenir les vitesses du DCA de la flotte principale
        _, _, forward_finished_fleet2 = control_algo.forward(t, N_fleet2, robots_poses_fleet2, np.array([[0.2,0],[0.,0]]), distance=2)
        # Copier directement la vitesse du DCA de la flotte principale
        vx_fleet2[0] = vx[0]  # La vitesse du DCA original
        vy_fleet2[0] = vy[0]
        
    elif not formation_finished_fleet2:
        r_ref_formation_fleet2 = np.array([[0,0],[0,0]])
        _, _, formation_finished_fleet2 = control_algo.formation(t, N_fleet2, robots_poses_fleet2, r_ref_formation_fleet2, np.array([[0.2,0],[0.,0]]), distance=4)
        vx_fleet2[0] = vx[0]
        vy_fleet2[0] = vy[0]
        
    elif not formation_bis_finished_fleet2:
        r_ref_formation_bis_fleet2 = np.array([[0,0],[0,0]])
        _, _, formation_bis_finished_fleet2 = control_algo.formation(t, N_fleet2, robots_poses_fleet2, r_ref_formation_bis_fleet2, np.array([[0.2,0],[0.,0]]), distance=6)
        vx_fleet2[0] = vx[0]
        vy_fleet2[0] = vy[0]
        
    elif not dance_finished_fleet2:
        _, _, dance_finished_fleet2 = control_algo.dance(t, N_fleet2, robots_poses_fleet2, np.array([[0.2,0],[0.,0]]), distance=6)
        vx_fleet2[0] = vx[0]
        vy_fleet2[0] = vy[0]
    
    # Le robot immobile (index 1) reste à position (0,0)
    vx_fleet2[1] = 0.0
    vy_fleet2[1] = 0.0
    
    # Appliquer les commandes aux robots
    for robotNo in range(fleet2.nbOfRobots):
        fleet2.robot[robotNo].ctrl = np.array([vx_fleet2[robotNo], vy_fleet2[robotNo]])
    
    # update data of simulation 
    simulation_fleet2.addDataFromFleet(fleet2)
    # integrate motion of the fleet
    fleet2.integrateMotion(Ts)

# plot animation
simulation.animation(figNo=1, pause=0.00001, robot_scale=0.2)   
simulation_fleet2.animation(figNo=5, pause=0.00001, robot_scale=0.2)  # Animation de la deuxième flotte

simulation_drone.plotXY(figNo=2)
# plot 2D trajectories
simulation.plotXY(figNo=2)
simulation_fleet2.plotXY(figNo=8)  # Tracé des trajectoires de la deuxième flotte
# plot on current figure the mission background for Question 1.6
#plot_mission_background()

# plot states' components Vs time
simulation.plotState(figNo=3)
simulation_drone.plotState(figNo=4)
simulation_fleet2.plotState(figNo=9)  # État des robots de la deuxième flotte

# plot control inputs' components Vs time
simulation.plotCtrl(figNo=6)
simulation_drone.plotCtrl(figNo=7)
simulation_fleet2.plotCtrl(figNo=10)  # Contrôle des robots de la deuxième flotte