#ifndef SRC_SIMULATPR_H_
#define SRC_SIMULATPR_H_
#include <cmath>
#include "OrbitalState.h"
#include <Eigen/Dense>

namespace Simulator {

// Predeclare this for use as friend to vehicle
class Simulator;

class Vehicle {
    friend Simulator;
 private:
    PV state;
    Eigen::Vector3d control {0.0, 0.0, 0.0};

 public:
    const OrbitalState& getState() const;

    PV getPv() const {
        return this->state;
    }

    COE getCoe() const {
        return pvToCoe(this->state);
    }
};

class Simulator {
 private:
    Vehicle target;
    Vehicle chaser;

 public:
    
};

} // namespace Simulator

#endif // SRC_SIMULATPR_H_
