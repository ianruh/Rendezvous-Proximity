#include "InfiniteLQRLinearTracking.h"
#include "Simulator.h"
#include "TrajectoryGeneration.h"
#include "Constants.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <optional>
#include <memory>
#include "RiccatiSolvers.h"

namespace Controllers {

InfiniteLQRLinearTrackingVehicle::InfiniteLQRLinearTrackingVehicle(
        double mass,
        Simulator::PV state,
        double targetSMA,
        std::shared_ptr<Trajectory> targetTrajectory
        ): Simulator::Vehicle(mass, state) {
    // Target SMA needed to construct the linearized A matrix
    this->targetSMA = targetSMA;
    this->targetTrajectory = targetTrajectory;
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

    Q = Eigen::Matrix<double, 6, 6>::Identity();
    Q.block(0,3,0,3) = Q.block(0,3,0,3) * (1.0/(1000.0*1000.0));
    Q.block(3,5,3,5) = Q.block(3,5,3,5) * (1.0/(1000.0*1000.0));
    
    R = 1.0/(0.001*0.001) * Eigen::Matrix<double, 3, 3>::Identity();

    auto P = solveRiccatiEigen(A, B, Q, R);
    if(!P) {
        throw std::runtime_error("Riccati Iteration failed to converge.");
    }

    this->K = this->R.inverse() * this->B.transpose() * P.value();
}

Eigen::Vector3d InfiniteLQRLinearTrackingVehicle::getControl(
        [[maybe_unused]] double t,
        [[maybe_unused]] const Simulator::Vehicle& target) {
    // Construct the control
    Simulator::Vector6d state = this->getRtn(target.getPv());
    Simulator::Vector6d targetState = this->targetTrajectory->getTargetState(t);
    Simulator::Vector6d error = state - targetState;
    
    Eigen::Vector3d control = -1 * this->K * error;

    // This control is acceleration, so f=ma
    return control*this->mass;
}

} // namespace Controllers
