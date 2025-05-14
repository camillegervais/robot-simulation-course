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

# dynamics of robots
# -------------------
robotDynamics_EP = 'unicycle'    # use 'singleIntegrator2D' or 'unicycle' #De base controler en integrateur simple mais on veut un robot unicycle
robotDynamics_TB3B = 'unicycle'           # use 'singleIntegrator2D' or 'unicycle'
robotDynamics_RMTT = 'singleIntegrator2D'          # use 'singleIntegrator2D' or 'unicycle'

robotDynamics = 'unicycle' # use 'signleIntegrator2D' or 'unicycle'
# ... initial positions randomly defined 
#initPositions = 20*np.random.rand(nbOfRobots,2)-10  # random init btw -10, +10
# ... initial positions defined from data    (dimension: nb of agents  x  2)
# id of robots: 
id = ['DCA','burger1', 'burger2', 'burger3', 'general_burger']
initPositions = np.array([[0, 0.4, 0.4, 0],      # x-coordinates (m)
                          [0, 0, -0.4,-0.4]]).T   # y-coordinates (m)
initPositions_general_burger = np.array([[0], [-1], [1]])      # x-coordinates (m)

# Formation of the fleet, 40 cm de distance entre les centres des robots, de haut en bas burger 1, 2, 3 

if (robotDynamics=='unicycle'):
    # orientation angles (rad)    (dimension: nb of agnts x 1)
    initAngles = np.array([[0., 0., 0., 0.]]).T
    initPoses = np.concatenate((initPositions, initAngles), axis=1)


# create fleet for formation control
if (robotDynamics=='singleIntegrator2D'):
    fleet = Fleet(nbRMEP + nbTB3B, dynamics=robotDynamics, initStates=initPositions)
else:
    fleet = Fleet(nbRMEP + nbTB3B, dynamics=robotDynamics, initStates=initPoses)
# create fleet for drone control
fleet_RMTT = Fleet(nbRMTT, dynamics=robotDynamics_RMTT, initStates=initPositions_general_burger) # if dynamics is unicycle, initStates should be a 2D array of size (nbOfRobots,3) with the last column being the angles of the robots as above



# sampling period for simulation
Ts = 0.01
# create simulation
simulation = FleetSimulation(fleet, t0=0.0, tf=10.0, dt=Ts)
simulation_drone = FleetSimulation(fleet_RMTT, t0=0.0, tf=10.0, dt=Ts)
formation_finished = False
forward_finished = False
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
        vx ,vy, forward_finished = control_algo.forward(t, N, robots_poses, np.array([[0.2,0],[0.,0],[0.,0],[0.,0]]), distance=2   )  # <= MODIFY CONTROL LAW IN "control_algo.py"
        
    elif forward_finished and not formation_finished:
        vx, vy, formation_finished = control_algo.formation(t, N, robots_poses, np.array([[0.2,0],[0.,0],[0.,0],[0.,0]]), distance=2   )  # <= MODIFY CONTROL LAW IN "control_algo.py"
        
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

# simulation_drone.animation(figNo=1, pause=0.00001, robot_scale=0.2)

# plot animation
simulation.animation(figNo=1, pause=0.00001, robot_scale=0.2)   

simulation_drone.plotXY(figNo=2)
# plot 2D trajectories
simulation.plotXY(figNo=2)
# plot on current figure the mission background for Question 1.6
#plot_mission_background()

# plot states' components Vs time
simulation.plotState(figNo=3)
simulation_drone.plotState(figNo=4)

# plot control inputs' components Vs time
simulation.plotCtrl(figNo=6)
simulation_drone.plotCtrl(figNo=7)

# plot 2D trajectories (every 'X steps' time instants
#simulation.plotXY(figNo=10, steps=50, links=True)
# plot on current figure the mission background for Question 1.6
#plot_mission_background()