# -*- coding: utf-8 -*-
"""
Simulation Class with 3D support

author: S. Bertrand, 2023
Modified for 3D visualization
"""
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.colors as mcolors
import lib.robot3d as robot_lib   # use this line for usage of this module by scripts from parent directory
#import robot as robot_lib      # use this line for usage of this module within its directory

# Custom function to animate 3D robot trajectories
def animate_3d(combined_simulation, combined_fleet, combined_ids=None):
    """Animate 3D robot trajectories from a simulation with minimal flicker"""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Set the plot limits and labels ONCE
    ax.set_xlim([-3, 10])
    ax.set_ylim([-3, 3])
    ax.set_zlim([0, 3])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.view_init(elev=20, azim=45)

    # Create a grid on the ground plane for better spatial reference
    x_grid = np.linspace(-3, 10, 14)
    y_grid = np.linspace(-3, 3, 7)
    X, Y = np.meshgrid(x_grid, y_grid)
    Z = np.zeros_like(X)
    ground = ax.plot_surface(X, Y, Z, color='gray', alpha=0.2)

    colors = ['red', 'green', 'blue', 'orange', 'purple', 'cyan', 'brown', 'pink']
    robot_lines = []
    robot_scatters = []
    vertical_lines = []
    legend_added = False

    # Pre-create line and scatter objects for each robot
    for robot_idx in range(combined_fleet.nbOfRobots):
        # Initial empty data
        line, = ax.plot([], [], [], color=colors[robot_idx % len(colors)], alpha=0.5)
        robot_lines.append(line)
        scatter = ax.scatter([], [], [], color=colors[robot_idx % len(colors)], s=80)
        robot_scatters.append(scatter)
        if robot_idx == 5:
            # For drone, add a vertical line
            vline, = ax.plot([], [], [], color=colors[robot_idx % len(colors)], linestyle='--', alpha=0.5)
            vertical_lines.append(vline)
        else:
            vertical_lines.append(None)

    # Animation loop
    for i in range(0, len(combined_simulation.t), 5):
        for robot_idx in range(combined_fleet.nbOfRobots):
            x = combined_simulation.robotSimulation[robot_idx].state[:i+1, 0]
            y = combined_simulation.robotSimulation[robot_idx].state[:i+1, 1]
            if combined_simulation.robotSimulation[robot_idx].robot.dynamics == 'singleIntegrator3D':
                z = combined_simulation.robotSimulation[robot_idx].state[:i+1, 2]
            else:
                z = np.zeros_like(x)
            # Update trajectory line
            robot_lines[robot_idx].set_data(x, y)
            robot_lines[robot_idx].set_3d_properties(z)
            # Update current position scatter
            current_x = x[-1]
            current_y = y[-1]
            current_z = z[-1]
            marker = 'o'
            size = 80
            if robot_idx == 0:
                marker = 's'; size = 100
            elif 1 <= robot_idx <= 3:
                marker = 'o'; size = 50
            elif robot_idx == 4:
                marker = '*'; size = 150
            elif robot_idx == 5:
                marker = '^'; size = 100
            else:
                marker = 'd'; size = 80
            # Remove and re-add scatter to update marker/size
            robot_scatters[robot_idx].remove()
            # --- FIX: Always add label for all robots, but only on first frame to avoid duplicate legend entries ---
            label = None
            if i == 0 and combined_ids and robot_idx < len(combined_ids):
                label = f"Robot {robot_idx}: {combined_ids[robot_idx]}"
            # Use a distinct marker for DCA_SI (robot 6)
            if robot_idx == 6:
                marker = 'X'
                size = 120
                color = 'black'
            # Use a distinct marker for Immobile (robot 7)
            if robot_idx == 7:
                marker = 'P'
                size = 120
                color = 'gray'
            robot_scatters[robot_idx] = ax.scatter(current_x, current_y, current_z, color=colors[robot_idx % len(colors)] if robot_idx not in [6,7] else (color), marker=marker, s=size, label=label)
            # Update vertical line for drone
            if robot_idx == 5 and vertical_lines[robot_idx] is not None:
                vertical_lines[robot_idx].set_data([current_x, current_x], [current_y, current_y])
                vertical_lines[robot_idx].set_3d_properties([0, current_z])
        if not legend_added:
            ax.legend(loc='upper right')
            legend_added = True
        ax.set_title(f"3D Robot Simulation - Time: {combined_simulation.t[i]:.2f}s")
        plt.pause(0.05)
    plt.show()

# Function to plot 3D trajectories
def plot_3d_trajectories(combined_simulation, combined_fleet, combined_ids=None):
    """Plot 3D trajectories from a simulation"""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Set the plot limits
    ax.set_xlim([-3, 10])
    ax.set_ylim([-3, 3])
    ax.set_zlim([0, 3])
    
    ax.set_xlabel('X position (m)', fontsize=12)
    ax.set_ylabel('Y position (m)', fontsize=12)
    ax.set_zlabel('Z position (m)', fontsize=12)
    ax.set_title('3D Robot Trajectories', fontsize=14)
    
    # Create a grid on the ground plane for better spatial reference
    x_grid = np.linspace(-3, 10, 14)
    y_grid = np.linspace(-3, 3, 7)
    X, Y = np.meshgrid(x_grid, y_grid)
    Z = np.zeros_like(X)
    ax.plot_surface(X, Y, Z, color='gray', alpha=0.2)
    
    # Colors for different robot types
    colors = ['red', 'green', 'blue', 'orange', 'purple', 'cyan', 'brown', 'pink']
    
    # Draw trajectories for each robot
    for robot_idx in range(combined_fleet.nbOfRobots):
        # Get the full trajectory
        x = combined_simulation.robotSimulation[robot_idx].state[:, 0]
        y = combined_simulation.robotSimulation[robot_idx].state[:, 1]
        
        # Get z-coordinate or use 0 if 2D robot
        if combined_simulation.robotSimulation[robot_idx].robot.dynamics == 'singleIntegrator3D':
            z = combined_simulation.robotSimulation[robot_idx].state[:, 2]
        else:
            z = np.zeros_like(x)
        
        # Plot trajectory with color gradient to show direction of movement
        points = np.array([x, y, z]).T.reshape(-1, 1, 3)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        # Create a line collection
        from matplotlib.collections import LineCollection
        lc = LineCollection(segments, cmap=plt.get_cmap('plasma'))
        lc.set_array(np.linspace(0, 1, len(x)))
        lc.set_linewidth(2)
        line = ax.add_collection3d(lc)
        
        # Mark start and end positions
        ax.scatter(x[0], y[0], z[0], color=colors[robot_idx % len(colors)], marker='o', s=100, label=f"Start: {combined_ids[robot_idx] if combined_ids and robot_idx < len(combined_ids) else f'Robot {robot_idx}'}")
        ax.scatter(x[-1], y[-1], z[-1], color=colors[robot_idx % len(colors)], marker='s', s=100)
        
        # For drone (robot_idx == 5), highlight its path
        if robot_idx == 5:
            # Add vertical lines to show altitude changes more clearly
            for i in range(0, len(x), 20):  # Show vertical reference every 20 points
                ax.plot([x[i], x[i]], [y[i], y[i]], [0, z[i]], color=colors[robot_idx % len(colors)], linestyle='--', alpha=0.3)
    
    # Add colorbar to show time progression
    cbar = plt.colorbar(line, ax=ax)
    cbar.set_label('Normalized Time', fontsize=10)
    
    # Add legend
    ax.legend(loc='upper right')
    
    # Add grid for better readability
    ax.grid(True)
    
    # Set a better viewing angle
    ax.view_init(elev=30, azim=45)
    
    plt.tight_layout()
    plt.show()
    
    # Also create multiple views of the same trajectory for better analysis
    fig, axs = plt.subplots(2, 2, figsize=(16, 12), subplot_kw={'projection': '3d'})
    axs = axs.flatten()
    
    view_angles = [(20, 0), (90, 0), (20, 90), (30, 45)]
    view_titles = ['Front View', 'Top View', 'Side View', 'Perspective View']
    
    for i, ax in enumerate(axs):
        # Set the plot limits
        ax.set_xlim([-3, 10])
        ax.set_ylim([-3, 3])
        ax.set_zlim([0, 3])
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(view_titles[i])
        
        # Set the specific view angle
        elev, azim = view_angles[i]
        ax.view_init(elev=elev, azim=azim)
        
        # Draw ground grid
        ax.plot_surface(X, Y, Z, color='gray', alpha=0.2)
        
        # Draw trajectories
        for robot_idx in range(combined_fleet.nbOfRobots):
            x = combined_simulation.robotSimulation[robot_idx].state[:, 0]
            y = combined_simulation.robotSimulation[robot_idx].state[:, 1]
            
            if combined_simulation.robotSimulation[robot_idx].robot.dynamics == 'singleIntegrator3D':
                z = combined_simulation.robotSimulation[robot_idx].state[:, 2]
            else:
                z = np.zeros_like(x)
                
            # Plot trajectory
            ax.plot(x, y, z, color=colors[robot_idx % len(colors)], alpha=0.7)
            
            # Mark start and end
            if i == 3:  # Only add legend in the last view
                robot_label = combined_ids[robot_idx] if combined_ids and robot_idx < len(combined_ids) else f'Robot {robot_idx}'
                ax.scatter(x[0], y[0], z[0], color=colors[robot_idx % len(colors)], marker='o', s=50, label=robot_label)
            else:
                ax.scatter(x[0], y[0], z[0], color=colors[robot_idx % len(colors)], marker='o', s=50)
