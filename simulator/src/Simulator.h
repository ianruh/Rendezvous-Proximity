#ifndef SRC_SIMULATPR_H_
#define SRC_SIMULATPR_H_
#include <cmath>
#include "OrbitalState.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <matplot/matplot.h>
#include <string>

namespace Simulator {

// Predeclare this for use as friend to vehicle
class Simulator;

class Vehicle {
    friend Simulator;
 private:
    double mass;
    PV state;

 public:
    Vehicle(double mass, PV state): mass(mass), state(state) {}

    Vehicle(const Vehicle& old) {
        this->state = old.state;
        this->mass = old.mass;
    }

    Vehicle(Vehicle&& other) = default;

    virtual ~Vehicle() {}

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

    void plotChaserRTN(matplot::axes_handle ax) const;
    void plotECI(matplot::axes_handle ax) const;
    void plotChaserControlVectors(matplot::axes_handle ax) const;
    void plotDistanceOverTime(matplot::axes_handle ax) const;
    void plotChaserControlOverTime(matplot::axes_handle ax) const;
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

    void integrate(double duration);
    
 public:
    Record record;

    Simulator(std::shared_ptr<Vehicle> target,
            std::shared_ptr<Vehicle> chaser,
            double controlFrequency = 10.0,
            double recordTimeStep = 100.0);

    void simulate(double duration,
            bool quiet = false);
    
};

} // namespace Simulator

#endif // SRC_SIMULATPR_H_
