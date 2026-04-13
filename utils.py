import numpy as np

def deg2rad(d):
    return np.deg2rad(d)

def rad2deg(r):
    return np.rad2deg(r)

def unit_vectors(theta_deg):
    """
    Local basis for each limb:
      e_r: radial direction in x-y plane
      e_t: tangential direction in x-y plane
      e_z: vertical axis
    """
    th = deg2rad(theta_deg)
    e_r = np.array([np.cos(th), np.sin(th), 0.0])
    e_t = np.array([-np.sin(th), np.cos(th), 0.0])
    e_z = np.array([0.0, 0.0, 1.0])
    return e_r, e_t, e_z

def wrap_to_pi(angle):
    """
    Wraps an angle in [rad] into the range (-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def select_alpha_solution(candidates_deg, prefer_deg=None):
    """
    Select one solution from a list of (typically 2) IK candidates.
    If prefer_deg is provided, pick the candidate closest to it.
    Otherwise, pick the first.
    """
    if not candidates_deg:
        return None
    if prefer_deg is None:
        return candidates_deg[0]
    
    idx = np.argmin([abs(c - prefer_deg) for c in candidates_deg])
    return candidates_deg[idx]
