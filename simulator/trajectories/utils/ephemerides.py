from astropy.time import Time
import numpy as np
from .propagator import Ephemeris
from .constants import *

moon_ephemeris = Ephemeris(
        pv=np.array([
            -3.099252869241486e05,
            2.060866920980158e05,
            1.283147528249122e05,
            -5.662599147083833e-01,
            -7.504504528567875e-01,
            -3.300536248679246e-01
        ], dtype=np.double)*1000.0,
        t=Time("2022-04-12").unix,
        central_body=EARTH)
