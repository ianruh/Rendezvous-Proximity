#include "InfiniteLQR.h"
#include "Simulator.h"
#include "Constants.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <optional>
#include "RiccatiSolvers.h"

namespace Controllers {

InfiniteLQRVehicle::InfiniteLQRVehicle(
        Simulator::PV state,
        double targetSMA,
        Eigen::Matrix<double, 6, 6> Q,
        Eigen::Matrix<double, 3, 3> R): Simulator::Vehicle(state) {
    // Target SMA needed to construct the linearized A matrix
    this->targetSMA = targetSMA;
    double n = std::sqrt(MU_EARTH/std::pow(this->targetSMA, 3));

    // System Dynamics
    A << 0.0,   0.0, 0.0,    1.0,  0.0, 0.0,
         0.0,   0.0, 0.0,    0.0,  1.0, 0.0,
         0.0,   0.0, 0.0,    0.0,  0.0, 1.0,
         3*n*n, 0.0, 0.0,    0.0,  2*n, 0.0,
         0.0,   0.0, 0.0,    -2*n, 0.0, 0.0,
         0.0,   0.0, -1*n*n, 0.0,  0.0, 0.0;

    // Control matrix
    B << 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0,
         0.0, 0.0, 0.0,
         1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    //Q = alpha * Eigen::Matrix<double, 6, 6>::Identity();
    //Q.block(0,0,3,3) = 1.0/(200.0*200.0) * Q.block(0,0,3,3);
    //Q.block(3,3,3,3) = 1.0/(10.0*10.0) * Q.block(3,3,3,3);
    //
    //R = beta * Eigen::Matrix<double, 3, 3>::Identity();
    //R = 1.0/(0.1*0.1) * R;

    auto P = solveRiccatiEigen(A, B, Q, R);
    if(!P) {
        throw std::runtime_error("Riccati Iteration failed to converge.");
    }

    this->K = R.inverse() * this->B.transpose() * P.value();
}

Eigen::Vector3d InfiniteLQRVehicle::getControl(
        [[maybe_unused]] double t,
        [[maybe_unused]] const Simulator::Vehicle& target) {
    // Construct the control
    Simulator::Vector6d state = this->getRtn(target.getPv());
    
    Eigen::Vector3d control = -1 * this->K * state;

    // This control is acceleration
    return control;
}

} // namespace Controllers
