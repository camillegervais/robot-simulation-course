# -*- coding: utf-8 -*-
"""
author: Sylvain Bertrand, 2023

   All variables are in SI units
    
   
   Variables used by the functions of this script
    - t: time instant (s)
    - robotNo: no of the current robot for which control is coputed (0 .. nbRobots-1)
    - poses:  size (3 x nbRobots)
        eg. of use: the pose of robot 'robotNo' can be obtained by: poses[:,robotNo]
            poses[robotNo,0]: x-coordinate of robot position (in m)
            poses[robotNo,1]: y-coordinate of robot position (in m)
            poses[robotNo,2]: orientation angle of robot (in rad)   (in case of unicycle dynamics only)
"""


import numpy as np
import math
import random



# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ==============
# all variables declared here will be known by functions below
# use keyword "global" inside a function if the variable needs to be modified by the function


# global toto

global firstCall   # this variable can be used to check the first call ever of a function
firstCall = True



# =============================================================================




# =============================================================================
def forward(t, robotNo, robots_poses, robots_velocities,distance=1.5):
# =============================================================================  

    # --- example of modification of global variables ---
    # ---(updated values of global variables will be known at next call of this funtion) ---
    # global toto
    # toto = toto +1
    global firstCall
    
    
    
    # number of robots (short notation)
    N = robots_poses.shape[0]
   
        
    # control law
    vx = 0.
    vy = 0.


    # adjacencdy matrix of communication graph
    # -----------------------------------------
    A= np.ones((N, N)) - np.eye(N)
    
    if (firstCall):  # print information (only once)
        print(A)
        
        
        firstCall = False
    
    kL = 2.0 # gain of the leader follower control law
    kF = 100.0 # gain of the formation control law
    

    # get positions of all robots
    X = robots_poses[:,0:2]
    X_d = robots_velocities[:,:] # velocity of the robots
    
    u = np.zeros((4,2))
    X_ref = np.array([[distance,0],[0,0],[0,0],[0,0]]) # reference position of the leader and followers

    # print("X_ref : ",X_ref)
    X_d_ref = np.array([[0.2,0],[0.2,0],[0.2,0],[0.2,0]]) # reference velocity of the leader
    r_ref = 0.4*np.array([[0,0],[1,0],[1,-1],[0,-1]] ) # reference distance between the followers and the leader
    r_d_ref = np.array([[0,0],[0,0],[0,0],[0,0]]) # reference velocity of the distance between the robots
    

    for i in range(N):
    # initialize control input vector
        if i == 0: # robot 3 : DCA is the leader
            u[i,:] = -kL*(X[i,:] - X_ref[i,:]) + X_d_ref[i,:]
        else:
            u[i,:] = -kF*( (X[i,:] - X[0,:]) - r_ref[i,:]) + X_d[0,:] + r_d_ref[i,:]
    
    # ===================== COMPUTATION OF ui =================================
        
    #                   IMPLEMENT CONTROL LAW HERE
    
    # =========================================================================
    
    
    # retrieve values from control input vector
    vx = u[:,0]
    vy = u[:,1]

    finished = False
    # check if the formation is finished
    for i in range(N):
        if X[0,0] > distance/3:
            finished = True
            break
    
    
    return vx, vy, finished
# =============================================================================

# =============================================================================
def formation(t, robotNo, robots_poses, r_ref, robots_velocities,distance=1.5):
# =============================================================================  

    # --- example of modification of global variables ---
    # ---(updated values of global variables will be known at next call of this funtion) ---
    # global toto
    # toto = toto +1
    global firstCall
    
    
    
    # number of robots (short notation)
    N = robots_poses.shape[0]
   
        
    # control law
    vx = 0.
    vy = 0.


    # adjacencdy matrix of communication graph
    # -----------------------------------------
    A= np.ones((N, N)) - np.eye(N)
    
    if (firstCall):  # print information (only once)
        print(A)
        
        
        firstCall = False
    
    kL = 5 # gain of the leader follower control law
    kF = 9 # gain of the formation control law
    kR = 3 # gain of the distance control law
    avoidance_radius = 0.4 # radius of the avoidance control law

    max_speed = 5 # maximum speed of the robots
    

    X = robots_poses[:,0:2]
    X_d = robots_velocities[:,:] # velocity of the robots
    
    u = np.zeros((4,2))
    X_ref = np.array([[distance,X[0,1]],[X[1,0],X[1,1]],[X[2,0],X[2,1]],[X[3,0],X[3,1]]]) # reference position of the leader and followers

    # print("X_ref : ",X_ref)
    X_d_ref = np.array([[0,0],[0,0],[0,0],[0,0]]) # reference velocity of the leader
    r_ref = r_ref # reference distance between the followers and the leader
    r_d_ref = np.array([[0,0],[0,0],[0,0],[0,0]]) # reference velocity of the distance between the robots
    # get positions of all robots
    

    for i in range(N):
    # initialize control input vector
        if i == 0: # robot 0 : DCA is the leader
            u[i,:] = -kL*(X[i,:] - X_ref[i,:]) + X_d_ref[i,:]
        else:
            u[i,:] = -kF*( (X[i,:] - X[0,:]) - r_ref[i,:]) + X_d[0,:] + r_d_ref[i,:]
        for j in range(N):
            if np.linalg.norm(X[i,:] - X[j,:]) < avoidance_radius ** 2 and i != j:
                u[i, :] += kR*(X[i,:] - X[j,:]) / np.linalg.norm(X[i,:] - X[j,:]) * (1 - np.linalg.norm(X[i,:] - X[j,:]) / avoidance_radius**2)**2
    
    # ===================== COMPUTATION OF ui =================================
        
    #                   IMPLEMENT CONTROL LAW HERE
    
    # =========================================================================
    
    
    # retrieve values from control input vector
    if np.linalg.norm(u) > max_speed:   
        u = u/np.linalg.norm(u) * max_speed
    vx = u[:,0]
    vy = u[:,1]
    
    finished = False
    # check if the formation is finished
    for i in range(N):
        if np.linalg.norm(vx) < 0.1 and np.linalg.norm(vy) < 0.1:
            finished = True
            break
    
    return vx, vy, finished
# =============================================================================


def dance(t, robotNo, robots_poses, robots_velocities,distance=1.5):
    return 0,0,False



# general template of a function defining a control law
# =============================================================================
def my_control_law(t, robotNo, robots_poses):
# =============================================================================  

    # --- example of modification of global variables ---
    # ---(updated values of global variables will be known at next call of this funtion) ---
    # global toto
    # toto = toto +1

    # number of robots
    nbOfRobots= robots_poses.shape[0]
    
    
    # control law
    vx = 0.
    vy = 0.

    # .................  TO BE COMPLETED HERE .............................
    
    return vx, vy
# =============================================================================

