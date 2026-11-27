# Functions for scenario definition in Python
import numpy as np


def time_vector(t_end,dt):
    # Defines a time vector of t_end duration and dt step
    t_vec = np.arange(0,t_end+dt,dt)
    return t_vec

def bump_velocity(t, height, length, speed, start_time):
    """Road vertical velocity for a raised-cosine bump (units: m/s).

    Height profile while the wheel is on the bump (x = speed * (t - start_time)):
        z_r = (height / 2) * (1 - cos(2 * pi * x / length))
    Differentiating with respect to time (dx/dt = speed):
        z_r_dot = (height * pi * speed / length) * sin(2 * pi * x / length)
    The velocity is zero before the bump starts and after it ends.
    """
    zr_dot = np.zeros_like(t)
    t_rel = t - start_time
    duration = length / speed
    on_bump = (t_rel >= 0) & (t_rel <= duration)
    x = speed * t_rel[on_bump]
    zr_dot[on_bump] = (height * np.pi * speed / length) * np.sin(2 * np.pi * x / length)
    return zr_dot

def make_road(name, sc, dt, comps=None):
    """Return (t, zr_dot) for one scenario. sc is that scenario's config dictionary."""
    t = time_vector(sc["t_end"], dt)
    if name == "bump":
        zr_dot = bump_velocity(t, sc["height_m"], sc["length_m"],
                               sc["speed_mps"], sc["lead_in_s"])
    else:
        raise ValueError(f"unsupported scenario: {name}")
    return t, zr_dot
