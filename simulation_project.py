from django.shortcuts import render
import pygame
import math
import random

SIMULATION_HEIGHT = 400
SIMULATION_WIDTH = 800
MARGIN = 50  # Margin for the simulation area

SIMULATION_COUNTER = 0  # Counter for the simulation time steps
SIMULATION_TIME = 0.0  # Simulation time in seconds

SCALE = 100  # Scale for converting real-world units to pixels

ROBOTS = []

def add_robot(name, initial_position, representation="default"):
    """
    Adds a robot with a given name, initial_position (x, y, z, theta) 
    and representation (string or any object).
    """
    ROBOTS.append({
        "name": name,
        "position": list(initial_position),
        "velocity": [0, 0, 0],  # v, w, vz
        "representation": representation,
        "velocities": []
    })

def render_robot(name, representation, screen):
    """
    Renders the robot on the screen using its representation.
    """
    for robot in ROBOTS:
        if robot["name"] == name:
            # Calculate scale factor based on z position (higher = bigger)
            scale_factor = 1 + robot["position"][2]  # Linear scaling with height
            base_size = 10  # Base size for shapes
            scaled_size = int(base_size * scale_factor)

            if representation == "burger":
                # Scaled brown circle for the burger
                pygame.draw.circle(screen, (139, 69, 19), 
                    (int(robot["position"][0] * SCALE) + MARGIN, 
                     int(robot["position"][1] * SCALE + SIMULATION_HEIGHT / 2) + MARGIN), 
                    scaled_size)
            elif representation == "drone":
                # Scaled red cross for the drone
                offset = scaled_size
                pos_x = int(robot["position"][0] * SCALE) + MARGIN
                pos_y = int(robot["position"][1] * SCALE + SIMULATION_HEIGHT / 2) + MARGIN
                pygame.draw.line(screen, (255, 0, 0), 
                    (pos_x + offset, pos_y + offset), 
                    (pos_x - offset, pos_y - offset), 2)
                pygame.draw.line(screen, (255, 0, 0), 
                    (pos_x + offset, pos_y - offset), 
                    (pos_x - offset, pos_y + offset), 2)
            elif representation == "canon":
                # Scaled blue square for the canon
                half_size = scaled_size // 2
                pos_x = int(robot["position"][0] * SCALE) + MARGIN - half_size
                pos_y = int(robot["position"][1] * SCALE + SIMULATION_HEIGHT / 2) + MARGIN - half_size
                pygame.draw.rect(screen, (0, 0, 255), 
                    (pos_x, pos_y, scaled_size, scaled_size))
            else:
                pygame.draw.circle(screen, (255, 0, 0), 
                    (int(robot["position"][0] * SCALE) + MARGIN, 
                     int(robot["position"][1] * SCALE + SIMULATION_HEIGHT / 2) + MARGIN), 
                    max(5, scaled_size // 2))

def set_velocities_list(name, velocities):
    """
    Sets the velocity list of a robot identified by its name.
    The givent velocities list must give at each time step the velocity of the robot in the format (v, w).
    """
    for robot in ROBOTS:
        if robot["name"] == name:
            robot["velocities"] = velocities
            break

def set_robot_velocity(name, v, w, vz):
    """
    Sets the velocity of a robot identified by its name.
    """
    for robot in ROBOTS:
        if robot["name"] == name:
            robot["velocity"] = [v, w, vz]
            break

def get_robot_position(name):
    """
    Returns the current position of the specified robot.
    """
    for robot in ROBOTS:
        if robot["name"] == name:
            return tuple(robot["position"])

def update_simulation(dt=1.0):
    """
    Updates the position of all robots based on their velocities.
    """

    global SIMULATION_COUNTER, SIMULATION_TIME

    SIMULATION_COUNTER += 1
    SIMULATION_TIME += dt

    for robot in ROBOTS:
        # Update velocity from the list if available
        robot["velocity"] = robot["velocities"].pop(0) if robot["velocities"] else robot["velocity"]
        # Unpack the velocity tuple
        v, w, vz = robot["velocity"]
        # Update orientation (theta)
        robot["position"][3] += w * dt  
        # Update position based on velocity and time step
        robot["position"][0] += v * math.cos(robot["position"][3]) * dt
        robot["position"][1] += v * math.sin(robot["position"][3]) * dt
        robot["position"][2] += vz
        # w is rotation around z axis, handle as needed

def render_background(screen):
    """
    Renders the background of the simulation area.
    """
    screen.fill((255, 255, 255))  # White background
    pygame.draw.rect(screen, (0, 0, 0), (MARGIN, MARGIN, SIMULATION_WIDTH, SIMULATION_HEIGHT), 2)  # Draw border

    # Draw grid lines
    for x in range(MARGIN, SIMULATION_WIDTH + MARGIN, SCALE):
        pygame.draw.line(screen, (200, 200, 200), (x, MARGIN), (x, SIMULATION_HEIGHT + MARGIN), 1)
    for y in range(MARGIN, SIMULATION_HEIGHT + MARGIN, SCALE):
        pygame.draw.line(screen, (200, 200, 200), (MARGIN, y), (SIMULATION_WIDTH + MARGIN, y), 1)

    # Display simulation counter and time
    font = pygame.font.Font(None, 36)
    counter_text = font.render(f'Steps: {SIMULATION_COUNTER}', True, (0, 0, 0))
    time_text = font.render(f'Time: {SIMULATION_TIME:.1f}s', True, (0, 0, 0))
    screen.blit(counter_text, (MARGIN, 20))
    screen.blit(time_text, (MARGIN + 200, 20))

def visualize_simulation():
    """
    Displays the positions of all robots in the simulation using pygame.
    """
    import pygame

    pygame.init()
    screen = pygame.display.set_mode((SIMULATION_WIDTH + 2 * MARGIN, SIMULATION_HEIGHT + 2 * MARGIN))
    clock = pygame.time.Clock()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False


        # Update robot positions based on their velocities      
        update_simulation(0.1)
        screen.fill((0, 0, 0))
        render_background(screen)
        for robot in ROBOTS:
            x, y, z, theta = robot["position"]
            # Simple 2D representation ignoring z
            render_robot(robot["name"], robot["representation"], screen)

        pygame.display.flip()
        clock.tick(45)

    pygame.quit()

if __name__ == "__main__":
    # Add robots to the simulation   
    add_robot("burger1", (0, 1, 0, 0), representation="burger")
    add_robot("burger2", (0, -1, 0, 0), representation="burger")
    add_robot("burger3", (1, 1, 0, 0), representation="burger")
    add_robot("DCA", (1, -1, 0, 0), representation="canon")
    add_robot("general_burger", (0, -1, 1, 0), representation="drone")
    add_robot("waffle", (0, 0, 0, 0), representation="default")


    # Set their velocities
    set_robot_velocity("burger1", 0.5, 0, 0)
    set_robot_velocity("burger2", 0.5, 0, 0)
    set_robot_velocity("burger3", 0.5, 0, 0)
    set_robot_velocity("DCA", 0.3, 0, 0)
    set_robot_velocity("general_burger", 0.3, 0, 0)
    set_robot_velocity("waffle", 0.3, 0, 0)

    # Set velocities list for the robots
    # Generate random velocity lists for each robot (100 timesteps)
    for robot_name in ["burger1", "burger2", "burger3", "DCA", "general_burger"]:
        velocities = []
        for _ in range(1000):
            v = random.uniform(0, 1)  # Random forward velocity between 0 and 1
            w = random.uniform(0, 2 * math.pi)  # Random angular velocity between -0.5 and 0.5
            vz = 0 if robot_name != "general_burger" else random.uniform(-0.2, 0.2)  # Vertical velocity only for drone
            velocities.append([v, w, vz])
        set_velocities_list(robot_name, velocities)

    # Start simulation
    visualize_simulation()
