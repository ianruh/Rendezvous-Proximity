import numpy as np
from scipy.optimize import fsolve
from .constants import mu_earth

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def lamberts(*, r1_vec: np.ndarray, r2_vec: np.ndarray, dt: float):
    """
    r1: [x, y, z] in meters
    r2: [x, y, z] in meters
    dt: time of flight in seconds

    returns: (pv1: np.ndarray, pv2: np.ndarray)
    """

    r1 = np.linalg.norm(r1_vec)
    r2 = np.linalg.norm(r2_vec)
    dTheta = angle_between(r1, r2)

    c = np.linalg.norm(r1_vec - r2_vec)
    s = (r1 + r2 + c)/2

    dtp = (1/3)*np.sqrt(2/mu_earth)*(
            s**(3/2) - np.sign(np.sin(dTheta)) * (s - c)**(3/2)
        )
    dtm = np.sqrt(s**3 / (8*mu_earth))*(
            np.pi - np.sqrt((s - c)/s) + np.sin(np.sqrt((s - c)/s))
        )

    def lamberts_eq(a):
        left = dt

        alpha = 2 * np.arcsin(np.sqrt(s / (2*a)))
        beta = 2 * np.arcsin(np.sqrt((s - c)/(2*a)))
        right = np.sqrt(a**3 / mu_earth) * (
                alpha - beta - (np.sin(alpha) - np.sin(beta))
            )
        return right - left

    a = fsolve(lamberts_eq, 1000e3)


    alpha = 2 * np.arcsin(np.sqrt(s / (2*a)))
    if(dt > dtm):
        alpha = 2*np.pi - alpha
    beta = 2 * np.arcsin(np.sqrt((s - c)/(2*a)))
    if(dTheta > np.pi):
        beta = -1*beta

    #p = (4*a*(s - r1)*(s - r2)/c**2)*np.sin((alpha + beta)/2)**2
    #e = np.sqrt(1 - p/a)

    u1_vec = unit_vector(r1_vec)
    u2_vec = unit_vector(r2_vec)
    uc_vec = (r2_vec - r1_vec)/c

    A = np.sqrt(mu_earth/(4*a))/np.tan(alpha/2)
    B = np.sqrt(mu_earth/(4*a))/np.tan(beta/2)

    v1 = (B + A)*uc_vec + (B - A)*u1_vec
    v2 = (B + A)*uc_vec - (B - A)*u2_vec

    pv1 = np.hstack((r1, v1))
    pv2 = np.hstack((r2, v2))

    import ipdb
    ipdb.set_trace()

    return (pv1, pv2)
