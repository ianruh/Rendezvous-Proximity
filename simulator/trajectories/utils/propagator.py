from scipy.integrate import RK45
import numpy as np
from .constants import *
import matplotlib.pyplot as plt
from functools import partial
from .orbital_elements import *
import PIL

class Ephemeris:

    def __init__(self,*, pv, t, central_body):
        self.pv0 = pv
        self.t0 = t
        self.central_body = central_body

        self.cache_times = np.array([t], dtype=np.double)
        self.cache_states = np.array([pv], dtype=np.double)

    def get_traj(self, *, t0: float, t1: float):
        """
        tf is a unix time stamp
        """
        return propagate(
                pv=self.get_pv(t=t0),
                t0=t0,
                t1=t1,
                central_body=self.central_body)


    def get_pv(self, *, t: float):
        """
        tf is a unix time stamp
        """
        # find the closest cache entry before t0
        i = np.argmin(np.abs(self.cache_times - t))
        i = max(i-1, 0)
        t0 = self.cache_times[i]
        pv0 = self.cache_states[i,:]

        times, states = propagate(
                pv=pv0,
                t0=t0,
                t1=t,
                central_body=self.central_body)
        
        if(t > self.cache_times[-1]):
            self.cache_times = np.hstack((self.cache_times, times))
            self.cache_states = np.vstack((self.cache_states, states))

        return states[-1,:]

    def plot(self, *,
            ax,
            t0,
            t1,
            label,
            pathColor="blue",
            pointColor="grey"):
        times, states = self.get_traj(t0=t0, t1=t1)
        ax.plot3D(states[:,0], states[:,1], states[:,2], label=f"{label} Trajectory", color=pathColor)
        
        ax.scatter3D(states[-1,0], states[-1,1], states[-1,2], label=label,color=pointColor);

    def plot_lunar_in_eci(self, *,
            ax,
            t0,
            t1,
            label,
            moon_ephemeris,
            pathColor="blue",
            pointColor="grey"):
        assert(self.central_body == MOON)
        lunar_times, lunar_states = self.get_traj(t0=t0, t1=t1)
        
        times = []
        states = []

        for i in range(lunar_times.shape[0]):
            times.append(lunar_times[i])
            states.append(pv_from_lunar_pv(
                lunar_pv=lunar_states[i,:],
                t=lunar_times[i],
                moon_ephemeris=moon_ephemeris))

        times = np.array(times, dtype=np.double)
        states = np.array(states, dtype=np.double).reshape((len(states), 6))

        ax.plot3D(states[:,0], states[:,1], states[:,2], label=f"{label} Trajectory", color=pathColor)
        
        ax.scatter3D(states[-1,0], states[-1,1], states[-1,2], label=label,color=pointColor);

    def plot_eci_in_lunar(self, *,
            ax,
            t0,
            t1,
            label,
            moon_ephemeris,
            pathColor="blue",
            pointColor="grey"):
        assert(self.central_body == EARTH)
        eci_times, eci_states = self.get_traj(t0=t0, t1=t1)
        
        times = []
        states = []

        for i in range(eci_times.shape[0]):
            times.append(eci_times[i])
            states.append(pv_to_lunar_pv(
                pv=eci_states[i,:],
                t=eci_times[i],
                moon_ephemeris=moon_ephemeris))

        times = np.array(times, dtype=np.double)
        states = np.array(states, dtype=np.double).reshape((len(states), 6))

        ax.plot3D(states[:,0], states[:,1], states[:,2], label=f"{label} Trajectory", color=pathColor)
        
        ax.scatter3D(states[-1,0], states[-1,1], states[-1,2], label=label,color=pointColor);

    @classmethod
    def earth_to_lunar(cls, *,
            eci_ephemeris,
            moon_ephemeris,
            t: float):
        """
        eci_ephemeris: The ephermis of the object in the ECI frame
        moon_ephemeris: The ephemeris of the moon in the ECI frame
        """
        assert(eci_ephemeris.central_body == EARTH)
        eci_pv = eci_ephemeris.get_pv(t=t)
        lunar_pv = pv_to_lunar_pv(pv=eci_pv, t=t, moon_ephemeris=moon_ephemeris)
        return Ephemeris(pv=lunar_pv, t=t, central_body=MOON)

    @classmethod
    def earth_from_lunar(cls, *,
            lunar_ephemeris,
            moon_ephemeris,
            t: float):
        """
        lunar_ephemeris: The ephermis of the object in the lunar frame
        moon_ephemeris: The ephermis of the moon in the ECI frame
        """
        assert(lunar_ephemeris.central_body == MOON)
        lunar_pv = lunar_ephemeris.get_pv(t=t)
        eci_pv = pv_from_lunar_pv(lunar_pv=lunar_pv, t=t, moon_ephemeris=moon_ephemeris)
        return Ephemeris(pv=eci_pv, t=t, central_body=EARTH)

def plot_earth(*, ax, label): 
    # load bluemarble with PIL
    bm = PIL.Image.open('earth.jpg')
    # it's big, so I'll rescale it, convert to array, and divide by 256 to get RGB values that matplotlib accept 
    bm = np.array(bm.resize([int(d/5) for d in bm.size]))/256.
    
    # coordinates of the image - don't know if this is entirely accurate, but probably close
    lons = np.linspace(-180, 180, bm.shape[1]) * np.pi/180 
    lats = np.linspace(-90, 90, bm.shape[0])[::-1] * np.pi/180 
    
    x = (EARTH.radius*0.9) * np.outer(np.cos(lons), np.cos(lats)).T
    y = (EARTH.radius*0.9) * np.outer(np.sin(lons), np.cos(lats)).T
    z = (EARTH.radius*0.9) * np.outer(np.ones(np.size(lons)), np.sin(lats)).T
    ax.plot_surface(x, y, z, rstride=4, cstride=4, facecolors = bm, label=label)

def plot_moon(*, ax, label): 
    # load bluemarble with PIL
    bm = PIL.Image.open('moon.jpg')
    # it's big, so I'll rescale it, convert to array, and divide by 256 to get RGB values that matplotlib accept 
    bm = np.array(bm.resize([int(d/5) for d in bm.size]))/256.
    
    # coordinates of the image - don't know if this is entirely accurate, but probably close
    lons = np.linspace(-180, 180, bm.shape[1]) * np.pi/180 
    lats = np.linspace(-90, 90, bm.shape[0])[::-1] * np.pi/180 
    
    x = (MOON.radius*0.9) * np.outer(np.cos(lons), np.cos(lats)).T
    y = (MOON.radius*0.9) * np.outer(np.sin(lons), np.cos(lats)).T
    z = (MOON.radius*0.9) * np.outer(np.ones(np.size(lons)), np.sin(lats)).T
    ax.plot_surface(x, y, z, rstride=4, cstride=4, facecolors = bm, label=label)

def dynamics(
        time: float,
        state: np.ndarray,
        central_body: CentralBody):
    """
    The time is in seconds.
    
    The state is [p | v]
    """
    r_vec = state[:3]
    vel = state[3:]
    
    r = np.linalg.norm(r_vec)
    
    a = -1 * central_body.mu / r**3 * r_vec

    return np.hstack((vel, a))

def propagate(*,
        pv: np.ndarray,
        t0: float,
        t1: float,
        central_body: CentralBody):
    bodyDynamics = partial(dynamics, central_body=central_body)
    rk45 = RK45(bodyDynamics,
            t0,
            pv,
            t1,
            atol=1e-6,
            rtol=1e-12)

    times = [t0]
    states = [pv]
    while(rk45.status == 'running'):
        rk45.step()
        times.append(rk45.t)
        states.append(rk45.y)
    
    times = np.array(times, dtype=np.double)
    states = np.array(states, dtype=np.double)

    return times, states
