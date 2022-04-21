#!/usr/bin/env python3

import unittest
import numpy as np
from astropy.time import Time
from .orbital_elements import *
from .constants import *
from .ephemerides import *

class TestOrbitalElements(unittest.TestCase):

    def test_pv_to_moe(self):
        pv = np.array([
            -6045e3,
            -3490e3,
            2500e3,
            -3.457e3,
            6.618e3,
            2.533e3
        ])
        moe_expected = np.array([
            1.71211182e-01,
            8.78808177e+06,
            2.67470361e+00,
            4.45546404e+00,
            3.50255117e-01,
            4.96472955e-01
        ])

        moe_got = pv_to_moe(pv=pv)
        for i in range(0, 6):
            np.testing.assert_approx_equal(moe_got[i], moe_expected[i], significant=5)

    def test_pv_from_moe(self):
        moe  = np.array([
            0.9,
            60000e3,
            80.0*np.pi/180.0,
            220.0*np.pi/180.0,
            70.0*np.pi/180.0,
            130.0*np.pi/180.0
        ])

        pv_expected = np.array([
            18437e3,
            17567.4e3,
            -9110.02e3,
            1.86458e3,
            2.41153e3,
            -3.67958e3
        ])

        pv_got = pv_from_moe(moe=moe)

        for i in range(0, 6):
            np.testing.assert_approx_equal(pv_got[i], pv_expected[i], significant=5)


    def test_pv_to_lunar_and_back(self):
        t = Time("2022-04-17").unix

        pv = np.array([
            -6045e3,
            -3490e3,
            2500e3,
            -3.457e3,
            6.618e3,
            2.533e3
        ])

        lunar_pv = pv_to_lunar_pv(pv=pv, t=t, moon_ephemeris=moon_ephemeris)
        received = pv_from_lunar_pv(lunar_pv=lunar_pv, t=t, moon_ephemeris=moon_ephemeris)
        
        for i in range(0, 6):
            np.testing.assert_approx_equal(pv[i], received[i], significant=5)

if __name__ == '__main__':
    unittest.main()
