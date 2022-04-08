#include "FiniteLQR.h"
#include "Simulator.h"
#include "Constants.h"
#include <cmath>

namespace Controllers {

FiniteLQRVehicle::FiniteLQRVehicle(
        double mass,
        Simulator::PV state,
        double targetSMA,
        double timeHorizon): Simulator::Vehicle(mass, state) {
    // Target SMA needed to construct the linearized A matrix
    this->targetSMA = targetSMA;
    this->timeHorizon = timeHorizon;
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

    double alpha = 1.0;
    Pf = alpha * Eigen::Matrix<double, 6, 6>::Identity();
    
    Q = Eigen::Matrix<double, 6, 6>::Zero();
    
    double beta = 1.0;
    R = beta * Eigen::Matrix<double, 3, 3>::Identity();
}

Eigen::Vector3d FiniteLQRVehicle::getControl(
        [[maybe_unused]] double t,
        [[maybe_unused]] const Simulator::Vehicle& target) const {
    return {0.0, 0.0, 0.0};
}

} // namespace Controllers
