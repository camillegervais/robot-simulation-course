# -*- coding: utf-8 -*-
"""
@author: Sylvain Bertrand, 2023


Simulation script of a Multi-Agent System

Agent dynamics can either be:
    -singleIntegrator2D:    x_dot = ux,   y_dot = uy
                            state : [x, y]
                            control input: [vx, vy]
    -unicycle:   x_dot = V.cos(theta),   y_dot = V.sin(theta),   theta_dot = omega
                            state : [x, y, theta]
                            control input: [V, omega] 
                            
    A conversion function is provided from single integrator inputs to unicycle inputs 
        [V, omega] = si_to_uni( [vx, vy], theta, kp=angular_speed_proportional_gain )

"""

import numpy as np
from lib.simulation import FleetSimulation
from lib.robot import Fleet, si_to_uni
import control_algo
from lib.mission import plot_mission_background 



# number of robots
# -----------------
nbOfRobots = 10

# dynamics of robots
# -------------------
robotDynamics = 'singleIntegrator2D'    # use 'signleIntegrator2D' or 'unicycle'


# initial states of robots 
# --------------------------

# ... initial positions randomly defined 
#initPositions = 20*np.random.rand(nbOfRobots,2)-10  # random init btw -10, +10

# ... initial positions defined from data    (dimension: nb of agents  x  2)
initPositions = np.array([[ -6, -5, -3, 2, 4, 6, 5,    1, -2,  -4],       # x-coordinates (m)
                          [0, 0.6, 2, 1.6, 1, -0.2, -1.4, -3, -3.6, -1.2]]).T   # y-coordinates (m)


# ... initial orientation angles and poses (USED FOR UNICYCLE DYNAMICS ONLY)
if (robotDynamics=='unicycle'):
    # orientation angles (rad)    (dimension: nb of agnts x 1)
    initAngles = np.array([[0., 0., 0., 0., 0., 0., 0., 0., 0., 0. ]]).T
    initPoses = np.concatenate((initPositions, initAngles), axis=1)



# create fleet
if (robotDynamics=='singleIntegrator2D'):
    fleet = Fleet(nbOfRobots, dynamics=robotDynamics, initStates=initPositions)
else:
    fleet = Fleet(nbOfRobots, dynamics=robotDynamics, initStates=initPoses)


# sampling period for simulation
Ts = 0.01

# create simulation
simulation = FleetSimulation(fleet, t0=0.0, tf=10.0, dt=Ts)

   
# simulation loop
for t in simulation.t:

    # get poses of robots as a single array of size (nb_of_robots, nb_of_states)
    robots_poses = fleet.getPosesArray()    

    # compute control input of each robot
    for robotNo in range(fleet.nbOfRobots):
        
        vx, vy = control_algo.formation(t, robotNo, robots_poses)     # <= MODIFY CONTROL LAW IN "control_algo.py"
        
        if (robotDynamics=='singleIntegrator2D'):
            fleet.robot[robotNo].ctrl = np.array([vx, vy]) 
        else:
            fleet.robot[robotNo].ctrl = si_to_uni(vx, vy, robots_poses[robotNo,2], kp=10.) 
    
    
    # update data of simulation 
    simulation.addDataFromFleet(fleet)

    # integrate motion of the fleet
    fleet.integrateMotion(Ts)


# plot animation
#simulation.animation(figNo=1, pause=0.00001, robot_scale=1.0)   

# plot 2D trajectories
simulation.plotXY(figNo=2)
# plot on current figure the mission background for Question 1.6
#plot_mission_background()


# plot states' components Vs time
simulation.plotState(figNo=3)

# plot control inputs' components Vs time
simulation.plotCtrl(figNo=6)

# plot 2D trajectories (every 'X steps' time instants
#simulation.plotXY(figNo=10, steps=50, links=True)
# plot on current figure the mission background for Question 1.6
#plot_mission_background()