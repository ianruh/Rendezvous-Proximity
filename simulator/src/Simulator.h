#ifndef SRC_SIMULATPR_H_
#define SRC_SIMULATPR_H_
#include <cmath>
#include "OrbitalState.h"
#include "TrajectoryGeneration.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <matplot/matplot.h>
#include <string>
#include <optional>

namespace Simulator {

// Predeclare this for use as friend to vehicle
class Simulator;

class Vehicle {
    friend Simulator;
 private:
    PV state;

 public:
    Vehicle(PV state): state(state) {}

    Vehicle(const Vehicle& old) {
        this->state = old.state;
    }

    Vehicle(Vehicle&& other) = default;

    virtual ~Vehicle() {}

    PV getPv() const { return this->state; }
    COE getCoe() const { return pvToCoe(this->state); }
    RTN getRtn(const PV& reference) const {
        return pvToRtn(this->state, reference);
    }

    void integrate(double duration, Eigen::Vector3d control);

    virtual Eigen::Vector3d getControl(
            [[maybe_unused]] double t,
            [[maybe_unused]] const Vehicle& target) {
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
    std::optional<std::vector<Vector6d>> trackedTrajectory;

    void write(const std::string& fileName) const;
    static Record load(const std::string& fileName);

    void plotChaserRTN(matplot::axes_handle ax) const;
    void plotChaserRTNState2D(matplot::axes_handle ax, size_t index) const;
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

    double controlSaturation; // acceleration in m/s/s

    double recordTimeStep;

    double time;

    void integrate(double duration);

    std::optional<std::shared_ptr<Controllers::Trajectory>> trackedTrajectory;
    
 public:
    Record record;

    Simulator(std::shared_ptr<Vehicle> target,
            std::shared_ptr<Vehicle> chaser,
            double controlFrequency = 10.0,
            double recordTimeStep = 1.0,
            double controlSaturation = 0.01);

    void setTrackedTrajectory(std::shared_ptr<Controllers::Trajectory> trackedTrajectory) {
        this->trackedTrajectory = trackedTrajectory;
    }

    void simulate(double duration,
            bool quiet = false);
    
};

} // namespace Simulator

#endif // SRC_SIMULATPR_H_
