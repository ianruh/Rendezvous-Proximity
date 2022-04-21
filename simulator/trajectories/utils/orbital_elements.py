import numpy as np
from .constants import *

def make_coe(*,e,a,i,raan,arg_peri,true_anom):
    return np.array([e,a,i,raan,arg_peri,true_anom], dtype=np.double)
def make_pv(*,x,y,z,vx,vy,vz):
    return np.array([x, y, z, vx, vy, vz], dtype=np.double)

def pv_to_moe(*,
        pv: np.ndarray,
        central_body: CentralBody = EARTH) -> np.ndarray:
    """Convert position-velocity to modified orbital elements.

    pv:  [x, y, z, vx, vy, vz] (meters, meters/second)
    moe: [e, a, i, raan, arg peri, true anom]

    """
    r_vec = pv[:3]
    r = np.linalg.norm(r_vec)
    v_vec = pv[3:]
    v = np.linalg.norm(v_vec)
    
    v_r = np.dot(r_vec, v_vec)/r

    # Basic vectors
    h_vec = np.cross(r_vec, v_vec)
    h = np.linalg.norm(h_vec)
    z_vec = np.array([0, 0, 1])
    n_vec = np.cross(z_vec, h_vec)
    n = np.linalg.norm(n_vec)
    e_vec = (v**2/central_body.mu - 1/r)*r_vec - (np.dot(r_vec, v_vec)/central_body.mu)*v_vec
    e = np.linalg.norm(e_vec)

    # Orbital elements
    i = np.arccos(h_vec[2]/h)

    Omega = np.arccos(n_vec[0]/n)
    if(n_vec[1] < 0):
        Omega = 2*np.pi - Omega

    omega = np.arccos(np.dot(n_vec, e_vec)/(n*e))
    if(e_vec[2] < 0):
        omega = 2*np.pi - omega

    f = np.arccos(np.dot(e_vec, r_vec)/(e*r))
    if(v_r < 0):
        f = 2*np.pi - f

    a = h**2 / (central_body.mu*(1 - e**2))

    return np.array([e, a, i, Omega, omega, f])

def pv_from_moe(*,
        moe: np.ndarray,
        central_body: CentralBody = EARTH) -> np.ndarray:
    """Convert modified orbital elements to position-velocity.

    pv:  [x, y, z, vx, vy, vz] (meters, meters/second)
    moe: [e, a, i, raan, arg peri, true anom]
    """
    e = moe[0]
    a = moe[1]
    i = moe[2]
    Omega = moe[3]
    omega = moe[4]
    f = moe[5]

    r = a*(1-e**2)/(1 + e * np.cos(f))

    r_pqw_vec = np.array([
        r*np.cos(f),
        r*np.sin(f),
        0.0
    ])

    v_pqw_vec = np.array([
        -1*np.sqrt(central_body.mu/(a*(1 - e**2))) * np.sin(f),
        np.sqrt(central_body.mu/(a*(1-e**2))) * (e + np.cos(f)),
        0.0
    ])

    T1 = np.array([
        np.cos(omega), -1*np.sin(omega), 0.0,
        np.sin(omega), np.cos(omega),    0.0,
        0.0,           0.0,              1.0
    ]).reshape((3,3))

    T2 = np.array([
        1.0, 0.0,       0.0,
        0.0, np.cos(i), -1*np.sin(i),
        0.0, np.sin(i), np.cos(i)
    ]).reshape((3,3))

    T3 = np.array([
        np.cos(Omega), -1*np.sin(Omega), 0.0,
        np.sin(Omega), np.cos(Omega),    0.0,
        0.0,           0.0,              1.0
    ]).reshape((3,3))

    r_eci_vec = (T3@T2@T1@r_pqw_vec.T).T
    v_eci_vec = (T3@T2@T1@v_pqw_vec.T).T

    return np.hstack((r_eci_vec, v_eci_vec))

def pv_to_altitude(*,
        pv: np.ndarray,
        central_body: CentralBody):
    """Get the alitiude of the given PV vector realtive to the central body"""
    return np.linalg.norm(pv[:3]) - central_body.radius

def pv_to_perigee_radius(*,
        pv: np.ndarray,
        central_body: CentralBody):
    """Get the perigee radius from the pv vector"""
    moe = pv_to_moe(pv=pv, central_body=central_body)
    return moe[1]*(1-moe[0])

def pv_to_apogee_radius(*,
        pv: np.ndarray,
        central_body: CentralBody):
    """Get the apogee radius from the pv vector"""
    moe = pv_to_moe(pv=pv, central_body=central_body)
    return moe[1]*(1+moe[0])

def pv_to_lunar_pv(*,
        pv: np.ndarray,
        t: float,
        moon_ephemeris) -> np.ndarray:
    """Convert an ECI PV vector to a lunar PV vector at the given time."""
    eci_pos = pv[:3]
    eci_vel = pv[3:]
    moon_eci_pv = moon_ephemeris.get_pv(t=t)
    moon_eci_pos = moon_eci_pv[:3]
    moon_eci_vel = moon_eci_pv[3:]
    
    lunar_pos = (MOON_FRAME_ROTATION_INV @ (eci_pos - moon_eci_pos).reshape((3,1))).reshape((3))
    lunar_vel = (MOON_FRAME_ROTATION_INV @ (eci_vel - moon_eci_vel).reshape((3,1))).reshape((3))

    return np.hstack((lunar_pos, lunar_vel))

def pv_from_lunar_pv(*,
        lunar_pv: np.ndarray,
        t: float,
        moon_ephemeris) -> np.ndarray:
    """Convert a lunar PV vector to an ECI PV vector at the given time."""
    lunar_pos = lunar_pv[:3]
    lunar_vel = lunar_pv[3:]
    moon_eci_pv = moon_ephemeris.get_pv(t=t)
    moon_eci_pos = moon_eci_pv[:3]
    moon_eci_vel = moon_eci_pv[3:]
    
    eci_pos = MOON_FRAME_ROTATION @ lunar_pos.reshape((3,1)).reshape(3) + moon_eci_pos
    eci_vel = MOON_FRAME_ROTATION @ lunar_vel.reshape((3,1)).reshape(3) + moon_eci_vel

    return np.hstack((eci_pos, eci_vel))

def print_moe(*, moe: np.ndarray) -> None:
    print(f"Eccentricity:    {moe[0]}")
    print(f"SMA:             {moe[1]} m")
    print(f"Inclination:     {moe[2] * 180.0 / np.pi} deg")
    print(f"Right Ascension: {moe[3] * 180.0 / np.pi} deg")
    print(f"Arg Perigee:     {moe[4] * 180.0 / np.pi} deg")
    print(f"True Anom:       {moe[5] * 180.0 / np.pi} deg")

def print_pv(*, pv: np.ndarray) -> None:
    print(f"Position: {pv[:3]} m")
    print(f"Velocity: {pv[3:]} m/s")
