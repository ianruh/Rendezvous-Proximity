#!/usr/bin/env python3

import unittest
import numpy as np
from .lamberts import *
from .constants import *

class TestLamberts(unittest.TestCase):

    def test_lamberts(self):
        r1 = np.array([5000e3, 10000e3, 2100e3])
        r2 = np.array([-14600e3, 2500e3, 7000e3])

        pv1, pv2 = lamberts(r1_vec=r1, r2_vec=r2, dt=60*60, central_body=EARTH)

        expectedv1 = np.array([-5.99249502e+03, 1.92536671e+03, 3.24563805e+03])
        expectedv2 = np.array([-3.31245850e+03, -4.19661901e+03, -3.85289060e+02])

        for i in range(3):
            np.testing.assert_approx_equal(expectedv1[i], pv1[3:][i], significant=5)
            np.testing.assert_approx_equal(expectedv2[i], pv2[3:][i], significant=5)

if __name__ == '__main__':
    unittest.main()
