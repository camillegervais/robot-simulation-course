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

# Variable pour vérifier le premier appel de chaque fonction
global firstCall_forward, firstCall_formation, firstCall_dance, firstCall_oscillation
firstCall_forward = True
firstCall_formation = True
firstCall_dance = True
firstCall_oscillation = True

# Variables for derivative correction in formation control
global prev_errors
global prev_time
prev_errors = np.zeros((5, 2))  # Initialisation pour 5 robots
prev_time = 0.0

# Variable to store the number of robots in the main fleet (needed for drone oscillation)
global N_fleet1
N_fleet1 = 5  # Default value, will be updated in the start.py file


# =============================================================================




# =============================================================================
def forward(t, robotNo, robots_poses, robots_velocities,distance=1.5):
# =============================================================================  

    # --- example of modification of global variables ---
    # ---(updated values of global variables will be known at next call of this funtion) ---
    # global toto
    # toto = toto +1
    global firstCall_forward
    
    
    
    # number of robots (short notation)
    N = robots_poses.shape[0]
   
        
    # control law
    vx = 0.
    vy = 0.


    # adjacencdy matrix of communication graph
    # -----------------------------------------
    A= np.ones((N, N)) - np.eye(N)
    
    if (firstCall_forward):  # print information (only once)
        print(A)
        
        
        firstCall_forward = False
    
    kL = 2.0 # gain of the leader follower control law
    kF = 100.0 # gain of the formation control law
    

    # get positions of all robots
    X = robots_poses[:,0:2]
    X_d = robots_velocities[:,:] # velocity of the robots
    
    u = np.zeros((N,2))  # Correction ici: utiliser N au lieu de 4 fixe
    
    # Créer une référence adaptée au nombre de robots
    X_ref = np.zeros((N, 2))
    X_ref[0] = [distance, 0]  # Position de référence pour le leader
    
    # Initialisation avec des zéros
    r_ref = np.zeros((N, 2))
    
    # Configuration des distances relatives adaptée dynamiquement au nombre de robots
    if N > 1:
        r_ref[1] = [1, 0]
    
    if N > 2:
        r_ref[2] = [1, -1]
    
    if N > 3:
        r_ref[3] = [0, -1]
    
    if N > 4:
        r_ref[4] = [8, 1.5]  # Position du waffle à l'opposé
    
    r_ref *= 0.4  # Appliquer l'échelle de 0.4
    
    # Référence de vitesses adaptée au nombre de robots
    X_d_ref = np.zeros((N, 2))
    for i in range(N):
        if i == 0:  # Pour le leader
            X_d_ref[i] = [0.2, 0]
    
    # Initialisation adaptée pour N robots
    r_d_ref = np.zeros((N, 2))
    

    for i in range(N):
    # initialize control input vector
        if i == 0: # robot 0 : DCA is the leader
            u[i,:] = -kL*(X[i,:] - X_ref[i,:]) + X_d_ref[i,:]
        elif i == N-1 and N > 4:  # Le robot waffle (seulement s'il existe)
            # Position fixe à l'opposé de la formation initiale
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
    global firstCall_formation
    
    # Variables pour stocker les erreurs précédentes (nécessaire pour le calcul dérivé)
    global prev_errors
    global prev_time
    
    # number of robots (short notation)
    N = robots_poses.shape[0]
   
    # control law
    vx = 0.
    vy = 0.

    # adjacencdy matrix of communication graph
    # -----------------------------------------
    A= np.ones((N, N)) - np.eye(N)
    
    if (firstCall_formation):  # print information (only once)
        print(A)
        print("Formation control with derivative correction activated")
        # Initialiser les variables pour le terme dérivé
        global prev_errors
        prev_errors = np.zeros((N, 2))
        global prev_time
        prev_time = t
        
        firstCall_formation = False
    
    # Gains pour le correcteur PD (Proportionnel-Dérivé)
    kL = 5     # gain of the leader follower control law (terme P)
    kF = 7     # gain of the formation control law (terme P)
    kD = 0.1   # gain dérivé pour limiter le dépassement (terme D)
    kR = 3     # gain of the distance control law
    avoidance_radius = 0.6  # radius of the avoidance control law

    max_speed = 5  # maximum speed of the robots
    
    # get positions of all robots
    X = robots_poses[:,0:2]
    X_d = robots_velocities[:,:] # velocity of the robots
    
    u = np.zeros((N,2))  # Correction ici: utiliser N au lieu de 4 fixe
    
    # Créer X_ref avec la bonne taille pour le nombre actuel de robots
    X_ref = np.zeros((N,2))
    X_ref[0] = [distance, X[0,1]]
    
    # Copier les positions actuelles pour les robots 1-3
    for i in range(1, N-1):
        X_ref[i] = [X[i,0], X[i,1]]
    
    # Position de référence pour le waffle (dernier robot)
    X_ref[N-1] = [X[N-1,0], X[N-1,1]]

    # print("X_ref : ",X_ref)
    X_d_ref = np.zeros((N,2))  # Correction: taille dynamique pour N robots
    
    # Gérer r_ref si c'est trop petit
    if len(r_ref) < N:
        # Étendre r_ref pour inclure le waffle
        r_ref_extended = np.zeros((N, 2))
        for i in range(len(r_ref)):
            r_ref_extended[i] = r_ref[i]
        # Position spécifique pour le waffle à l'opposé
        r_ref_extended[N-1] = [8, 1.5]
        r_ref = r_ref_extended
        
    r_d_ref = np.zeros((N,2))  # Correction: taille dynamique pour N robots
    
    # Tableaux pour stocker les erreurs actuelles
    current_errors = np.zeros((N, 2))
    
    # Calculer le dt pour la dérivation
    dt = t - prev_time
    if dt < 0.0001:  # Éviter division par zéro
        dt = 0.0001
    
    # Pour chaque robot
    for i in range(N):
        # initialize control input vector
        if i == 0:  # robot 0 : DCA is the leader
            # Calcul de l'erreur
            error = X[i,:] - X_ref[i,:]
            current_errors[i,:] = error
            
            # Calcul du terme dérivé (vitesse de l'erreur)
            # error_derivative = (error - prev_errors[i,:]) / dt
            error_derivative = 0
            
            # Correction PD pour le leader
            u[i,:] = -kL * error - kD * error_derivative + X_d_ref[i,:]
            
        elif i == N-1:  # Robot waffle - reste immobile
            u[i,:] = np.zeros(2)  # Le waffle ne bouge pas
            current_errors[i,:] = np.zeros(2)  # Pas d'erreur pour waffle
            
        else:  # Pour les followers (robots burger)
            # Erreur de formation par rapport au leader avec la position relative désirée
            error = (X[i,:] - X[0,:]) - r_ref[i,:]
            current_errors[i,:] = error
            
            # Calcul du terme dérivé (vitesse de l'erreur)
            error_derivative = (error - prev_errors[i,:]) / dt
            
            # Correction PD pour les followers
            # Note: on ajoute X_d[0,:] pour que les followers suivent la vitesse du leader
            u[i,:] = -kF * error - kD * error_derivative + X_d[0,:] + r_d_ref[i,:]
          # Appliquer l'évitement de collision pour tous les followers (sauf le waffle)
        # Le DCA (i=0) ne subit pas de répulsion, ce sont les autres robots qui doivent l'éviter
        if i != 0 and i != N-1:  # Ni le leader (DCA) ni le waffle
            for j in range(N):
                if np.linalg.norm(X[i,:] - X[j,:]) < avoidance_radius ** 2 and i != j:
                    u[i, :] += kR*(X[i,:] - X[j,:]) / np.linalg.norm(X[i,:] - X[j,:]) * (1 - np.linalg.norm(X[i,:] - X[j,:]) / avoidance_radius**2)**2
    
    # Sauvegarder les erreurs actuelles pour le prochain pas de temps
    prev_errors = current_errors.copy()
    prev_time = t
    
    # Limitation de vitesse
    norm_u = np.linalg.norm(u, axis=1)
    for i in range(N):
        if norm_u[i] > max_speed:
            u[i,:] = u[i,:] / norm_u[i] * max_speed
    
    vx = u[:,0]
    vy = u[:,1]
    
    finished = False
    # check if the formation is finished
    if np.all(np.linalg.norm(u[:-1], axis=1) < 0.1):  # Tous les robots sauf waffle sont presque immobiles
        finished = True
    
    return vx, vy, finished
# =============================================================================


def dance(t, robotNo, robots_poses, robots_velocities, distance=1.5):
    # --- example of modification of global variables ---
    global firstCall_dance
    
    # number of robots (short notation)
    N = robots_poses.shape[0]
    
    # control law
    vx = np.zeros(N)
    vy = np.zeros(N)
    
    if (firstCall_dance):  # print information (only once)
        print("Starting dance routine around waffle robot")
        firstCall_dance = False
    
    # Position du robot qui sera le centre de la danse
    # Si N >= 5, c'est le waffle (dernier robot), sinon c'est le dernier robot disponible
    center_position = robots_poses[N-1, 0:2]
    
    # Paramètres pour la danse circulaire
    radius = 0.6  # rayon du cercle de danse
    angular_velocity = 0.5  # vitesse de rotation autour du cercle
    phase_shift = 2 * np.pi / max(1, (N-2))  # décalage de phase adaptable
    
    # Paramètres pour la force de répulsion
    repulsion_radius = 0.35  # rayon d'action de la force de répulsion
    repulsion_gain = 2.0  # gain de la force de répulsion
    
    # Initialisation des vitesses
    u = np.zeros((N, 2))
    
    # Positions des robots
    X = robots_poses[:, 0:2]
    
    # Pour chaque robot
    for i in range(N):
        if i == 0 or i == N-1:
            # Le robot 0 (DCA/DCA_SI) et le dernier robot (waffle/Immobile) restent immobiles
            u[i,:] = np.array([0.0, 0.0])
        else:
            # Pour les robots intermédiaires
            # 1. Calcul de la force d'attraction vers la position cible
            
            # Angle dans le cercle, variant avec le temps et un décalage pour chaque robot
            angle = angular_velocity * t + (i-1) * phase_shift
            
            # Position cible sur le cercle
            target_position = center_position + radius * np.array([np.cos(angle), np.sin(angle)])
            
            # Vecteur direction vers la cible
            direction = target_position - X[i,:]
            
            # Normalisation et ajustement de la vitesse
            distance_to_target = np.linalg.norm(direction)
            
            # Gain proportionnel pour suivre la trajectoire circulaire
            kp = 3.0
            
            # Force d'attraction vers la cible
            attraction_force = np.zeros(2)
            if distance_to_target > 0.01:  # Éviter division par zéro
                attraction_force = kp * direction
            else:
                # Si on est proche de la cible, on applique directement une vitesse tangentielle
                tangent = np.array([-np.sin(angle), np.cos(angle)])
                attraction_force = angular_velocity * radius * tangent
            
            # 2. Calcul des forces de répulsion entre robots intermédiaires
            repulsion_force = np.zeros(2)
            
            # Parcourir tous les autres robots intermédiaires
            for j in range(1, N-1):  # Indices adaptés à N
                if i != j:  # Ne pas se repousser soi-même
                    # Vecteur de distance entre les deux robots
                    dist_vector = X[i,:] - X[j,:]
                    dist = np.linalg.norm(dist_vector)
                    
                    # Appliquer une force de répulsion inversement proportionnelle à la distance
                    if dist < repulsion_radius and dist > 0:  # Éviter division par zéro
                        # Force de répulsion entre robots, plus forte quand ils sont proches
                        repel = repulsion_gain * (1.0 - dist/repulsion_radius)**2
                        # Direction normalisée
                        if dist > 0.001:  # Éviter division par zéro
                            repulsion_force += repel * dist_vector / dist
                        else:
                            # Si les robots sont très proches, ajouter une petite perturbation aléatoire
                            repulsion_force += repel * np.array([np.random.uniform(-1, 1), np.random.uniform(-1, 1)])
            
            # 3. Combiner les forces d'attraction et de répulsion
            u[i,:] = attraction_force + repulsion_force
    
    # Limitation de vitesse
    max_speed = 2.0
    for i in range(N):
        speed = np.linalg.norm(u[i,:])
        if speed > max_speed:
            u[i,:] = u[i,:] / speed * max_speed
    
    # Extraction des composantes de vitesse
    vx = u[:,0]
    vy = u[:,1]
    
    # La danse ne se termine jamais automatiquement
    finished = False
    
    return vx, vy, finished


# =============================================================================
def drone_oscillation(t, robotNo, robots_poses, robots_velocities, oscillation_start_time=0, distance=1.5):
# =============================================================================  
    # --- example of modification of global variables ---
    global firstCall_oscillation
    
    # number of robots (short notation)
    N = robots_poses.shape[0]
    
    # control law
    vx = np.zeros(N)
    vy = np.zeros(N)
    
    # Vérifier le premier appel de la fonction
    if 'firstCall_oscillation' not in globals():
        global firstCall_oscillation
        firstCall_oscillation = True
    
    if (firstCall_oscillation):  # print information (only once)
        print("Starting drone oscillation phase above waffle robot")
        firstCall_oscillation = False
    
    # Position du waffle (dernier robot de la flotte principale)
    waffle_idx = 4  # Index du robot waffle (5ème robot, indice 4)
    waffle_position = robots_poses[waffle_idx, 0:2]
    
    # Position actuelle du drone RMTT
    rmtt_idx = N_fleet1  # Le drone est juste après la flotte principale
    drone_position = robots_poses[rmtt_idx, 0:2]
    
    # Paramètres de l'oscillation
    amplitude = 1  # Amplitude de l'oscillation en mètres
    frequency = 0.8  # Fréquence de l'oscillation en Hz
    
    # Temps relatif depuis le début de l'oscillation
    relative_time = t - oscillation_start_time
    
    # Nombre de cycles complets pour terminer (2 oscillations)
    num_cycles = 5
    cycle_duration = 1.0 / frequency  # Durée d'un cycle complet
    total_oscillation_time = num_cycles * cycle_duration
    
    # Vérifier si les deux oscillations sont terminées
    oscillations_complete = relative_time >= total_oscillation_time
    
    # Position cible du drone
    if not oscillations_complete:
        # Pendant l'oscillation: au-dessus du waffle avec oscillation sinusoïdale en x
        target_x = waffle_position[0] + amplitude * np.sin(2 * np.pi * frequency * relative_time)
        target_y = waffle_position[1]
    else:
        # À la fin des oscillations: retour à la position initiale (au-dessus du waffle sans oscillation)
        target_x = waffle_position[0]
        target_y = waffle_position[1]
    
    target_position = np.array([target_x, target_y])
    
    # Contrôle proportionnel pour atteindre la position cible
    kp = 3.0  # Gain proportionnel
    
    # Pour tous les robots
    for i in range(N):
        if i == rmtt_idx:  # Seulement pour le drone RMTT
            # Différence entre position cible et position actuelle
            error = target_position - drone_position
            
            # Appliquer un contrôle proportionnel
            u_drone = kp * error
            
            # Limiter la vitesse maximale
            max_speed = 2.0
            speed = np.linalg.norm(u_drone)
            if speed > max_speed:
                u_drone = u_drone / speed * max_speed
            
            # Assigner les vitesses au drone
            vx[i] = u_drone[0]
            vy[i] = u_drone[1]
            print(vx[i], vy[i])
        else:
            # Tous les autres robots restent immobiles
            vx[i] = 0.0
            vy[i] = 0.0
    
    # Vérifier si la phase d'oscillation est terminée:
    # 1. Les oscillations sont terminées
    # 2. Le drone est retourné près de sa position cible finale (au-dessus du waffle)
    finished = oscillations_complete and np.linalg.norm(error) < 2
    
    if finished:
        print("Drone oscillation phase complete - returning to normal operation")
    
    return vx, vy, finished


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

