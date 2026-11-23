# Functions for scenario definition in Python
import numpy as np


def time_vector(t_end,dt):
    # Defines a time vector of t_end duration and dt step
    t_vec = np.arange(0,t_end+dt,dt)
    return t_vec
