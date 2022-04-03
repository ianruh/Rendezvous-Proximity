#include "gtest/gtest.h"
#include "OrbitalState.h"
#include "Constants.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Simulator;

TEST(OrbitalStateTests, PV_TO_COE) { 
    PV pv {
        -6045e3,
        -3490e3,
        2500e3,
        -3.457e3,
        6.618e3,
        2.533e3
    };
    COE coe_expected {
        1.71211182e-01,
        8.78808177e+06,
        2.67470361e+00,
        3.50255117e-01,
        4.45546404e+00,
        4.96472955e-01
    };

    COE recieved = pvToCoe(pv);

    EXPECT_TRUE(recieved.isApprox(coe_expected, 0.0001));
}

TEST(OrbitalStateTests, PV_FROM_COE) {
    COE coe  {
        0.9,
        60000e3,
        80.0*PI/180.0,
        70.0*PI/180.0,
        220.0*PI/180.0,
        130.0*PI/180.0
    };
    
    PV pv_expected {
        18437e3,
        17567.4e3,
        -9110.02e3,
        1.86458e3,
        2.41153e3,
        -3.67958e3
    };

    PV recieved = pvFromCoe(coe);

    EXPECT_TRUE(recieved.isApprox(pv_expected, 0.0001));
}
