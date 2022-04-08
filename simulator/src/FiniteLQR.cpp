#include "FiniteLQR.h"
#include "Simulator.h"
#include "Constants.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <boost/numeric/odeint.hpp>

namespace odeint = boost::numeric::odeint;

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
    
    double gamma = 1.0;
    Q = gamma * Eigen::Matrix<double, 6, 6>::Identity();
    
    double beta = 0.01;
    R = beta * Eigen::Matrix<double, 3, 3>::Identity();
}

Eigen::Vector3d FiniteLQRVehicle::getControl(
        [[maybe_unused]] double t,
        [[maybe_unused]] const Simulator::Vehicle& target) {
    // Perform the riccati iteration to get P for the next 1000s
    this->riccatiIteration(t, t+this->timeHorizon);
    // Construct the control
    Simulator::Vector6d state = this->getRtn(target.getPv());
    
    // Get the P matrix. Since we are doing iterations on every control cycle,
    // this is always the last one. Otherwise it would need to be interpolated.
    Eigen::Matrix<double, 6, 6> P = this->Pt[this->Pt.size()-1];

    Eigen::Vector3d control = -1 * this->R.inverse() * this->B.transpose() *
        P*state;
    // This control is acceleration, so f=ma
    return control*this->mass;
}

using Stepper = odeint::runge_kutta_dopri5<
    FiniteLQRVehicle::state_type,
    double,
    FiniteLQRVehicle::state_type,
    double,
    odeint::vector_space_algebra>;
void FiniteLQRVehicle::riccatiIteration(
        double t0,
        double tf) {

    auto differentialRicatti = [this](
            const FiniteLQRVehicle::state_type& P,
            FiniteLQRVehicle::state_type &dpdt,
            [[maybe_unused]] double t) {
        dpdt = -1 * this->A.transpose()*P - P*this->A + 
        P * this->B * this->R.inverse() * this->B.transpose() * P -
        this->Q;
        std::cout << "P: \n";
        std::cout << P << '\n';
        std::cout << "dP/dt: \n";
        std::cout << dpdt << '\n';
    };

    auto writeSolution = [this](
            const FiniteLQRVehicle::state_type& p,
            const double t) {
        this->Pt.push_back(p);
        this->times.push_back(t);
    };

    double stepSize = -0.1;
    odeint::integrate_adaptive(
            Stepper(),
            differentialRicatti,
            this->Pf,
            tf,
            t0,
            stepSize,
            writeSolution);
    exit(0);
}

Eigen::Matrix<double, 6, 6> FiniteLQRVehicle::interpolateP(double t) const {
    size_t t1Index = 0;
    for(size_t i = 0; i < this->times.size(); i++) {
        if(t < this->times[i]) {
            t1Index = i;
            break;
        }
    }
    size_t t0Index = t1Index + 1;
    if(t0Index >= this->times.size()) {
        return this->Pt[0];
    }

    double t0 = this->times[t0Index];
    double t1 = this->times[t1Index];
    Eigen::Matrix<double, 6, 6> P0 = this->Pt[t0Index];
    Eigen::Matrix<double, 6, 6> P1 = this->Pt[t1Index];

    Eigen::Matrix<double, 6, 6> P = P0 + (t-t0)/(t1-t0)*(P1-P0);
    return P;
}

} // namespace Controllers
