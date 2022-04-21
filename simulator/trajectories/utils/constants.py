import numpy as np

class CentralBody:

    def __init__(
            self,
            *,
            name,
            mass,
            radius,
            gravitational_parameter,
            soi):
        self.name = name
        self.mass = mass
        self.radius = radius
        self.mu = gravitational_parameter
        self.soi = soi

EARTH = CentralBody(
        name="Earth",
        mass=5.97e24, # kg
        radius=6378e3, # meters
        gravitational_parameter=3.986004418e14, # using meters
        soi=9.24e8 # meters
)
EARTH.obiquity = 23.45 * np.pi/180.0 # radians

MOON = CentralBody(
        name="Moon",
        mass=73.48e21, # kg
        radius=1737.5e3, # m
        gravitational_parameter=4.9048695e12, # using meters
        soi=6.61e7 # meters
)

# Relative to ECI at J2000. lunar_frame = MOON_FRAME_ROTATION * eci_frame
# TODO(ianruh): Find the actual rotation
MOON_FRAME_ROTATION = np.array([
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
], dtype=np.double).reshape(3,3)
MOON_FRAME_ROTATION_INV = np.linalg.inv(MOON_FRAME_ROTATION)
