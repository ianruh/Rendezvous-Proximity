#ifndef SRC_TRAJECTORY_TRACKING_H_
#define SRC_TRAJECTORY_TRACKING_H_

#include <cmath>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include "OrbitalState.h"

namespace Controllers {

class Trajectory {
 public:
    virtual ~Trajectory() {}
    
    virtual Simulator::RTN getTargetState(double t) = 0;
};

class LinearWaypointTrajectory: public Trajectory {
 private:
    std::vector<double> waypointTimes;
    std::vector<Simulator::Vector6d> waypoints;

 public:
    LinearWaypointTrajectory(
            std::vector<double> times,
            std::vector<Simulator::Vector6d> waypoints) {
        this->waypointTimes = times;
        this->waypoints = waypoints;

        assert(times.size() == waypoints.size());
    }

    LinearWaypointTrajectory(const LinearWaypointTrajectory& old) {
        this->waypointTimes = old.waypointTimes;
        this->waypoints = old.waypoints;
    }

    LinearWaypointTrajectory(LinearWaypointTrajectory&& other) = default;

    Simulator::RTN getTargetState(double t) override {
        size_t t1Index = std::upper_bound(
                this->waypointTimes.begin(),
                this->waypointTimes.end(),
                t) - this->waypointTimes.begin();

        // If t is greater than all of our waypoint times, return the last
        // waypoint
        if(t1Index == this->waypointTimes.size()) {
            return this->waypoints.at(t1Index-1);
        }

        // If t is less than all of the times, return the first one.
        if(t1Index == 0) {
            return this->waypoints.at(0);
        }

        // This is safe, since we checked that t1Index > 0
        size_t t0Index = t1Index - 1;

        // Now we can find the linear interpolant
        double t0 = this->waypointTimes.at(t0Index);
        double t1 = this->waypointTimes.at(t1Index);

        Simulator::Vector6d state0 = this->waypoints.at(t0Index);
        Simulator::Vector6d state1 = this->waypoints.at(t1Index);

        Simulator::Vector6d targetState = state0 + (t - t0)/(t1 - t0)*(state1 - state0);

        return targetState;
    }
};

}

#endif
