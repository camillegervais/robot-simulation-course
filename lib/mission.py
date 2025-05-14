# -*- coding: utf-8 -*-
"""
Plot function for multi robot mission

Author: S. Bertrand, 2024
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle



# -----------------------------------------------------------------------------
def plot_mission_background():
# -----------------------------------------------------------------------------     
    
    plt.gca().add_patch(Rectangle((-1,1), 2, 9, facecolor='gray'))    
    plt.gca().add_patch(Rectangle((-1,-10), 2, 9, facecolor='gray'))
    plt.gca().add_patch(Circle((7.5,1),0.5, edgecolor='k', facecolor='orange'))
    plt.gca().add_patch(Circle((7.5,-1),0.5, edgecolor='k', facecolor='orange'))
    plt.gca().add_patch(Circle((6.5,0),0.5, edgecolor='k', facecolor='orange'))
    plt.gca().add_patch(Circle((8.5,0),0.5, edgecolor='k', facecolor='orange'))
    plt.plot([-9],[1], 'r+')
    plt.plot([-7],[1], 'g+')
    plt.plot([-9],[-1], 'b+')
    plt.plot([-7],[-1], 'y+')
    