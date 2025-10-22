#!/usr/bin/env python3
import numpy as np
from robot_sim.visualize import animate_leg

# Simple periodic motion for testing
phi1_func = lambda t: 0.5*np.sin(2*np.pi*0.5*t)
phi4_func = lambda t: -0.5*np.sin(2*np.pi*0.5*t + np.pi/6)

if __name__ == "__main__":
    animate_leg(phi1_func, phi4_func, duration=6.0, fps=30)
