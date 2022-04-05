#ifndef SRC_SIMULATPR_H_
#define SRC_SIMULATPR_H_
#include <cmath>
#include "OrbitalState.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace Simulator {

// Predeclare this for use as friend to vehicle
class Simulator;

class Vehicle {
    friend Simulator;
 private:
    PV state;
    double mass;

 public:
    Vehicle(double mass): mass(mass) {}

    Vehicle(const Vehicle& old) {
        this->state = old.state;
        this->mass = old.mass;
    }

    Vehicle(Vehicle&& other) = default;

    PV getPv() const { return this->state; }
    COE getCoe() const { return pvToCoe(this->state); }

    void integrate(double duration, Eigen::Vector3d control);

    virtual Eigen::Vector3d getControl(
            [[maybe_unused]] double t,
            [[maybe_unused]] const Vehicle& target) const {
        return {0.0, 0.0, 0.0};
    }
};


class Record {
 public:
    std::vector<double> times;
    std::vector<PV> targetState;
    std::vector<PV> chaserState;
    std::vector<Eigen::Vector3d> targetControl;
    std::vector<Eigen::Vector3d> chaserControl;
};

class Simulator {
 private:
    std::shared_ptr<Vehicle> target;
    std::shared_ptr<Vehicle> chaser;
    Eigen::Vector3d targetControl;
    Eigen::Vector3d chaserControl;

    double controlFrequency;
    double controlTimestep;

    double recordTimeStep;

    double time;

    Record record;

    void integrate(double duration);
    
 public:

    Simulator(std::shared_ptr<Vehicle> target,
            std::shared_ptr<Vehicle> chaser,
            double controlFrequency = 10.0,
            double recordTimeStep = 100.0);

    void simulate(double duration,
            bool quiet = false);
    
};

} // namespace Simulator

#endif // SRC_SIMULATPR_H_
