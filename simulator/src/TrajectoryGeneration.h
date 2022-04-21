#ifndef SRC_TRAJECTORY_TRACKING_H_
#define SRC_TRAJECTORY_TRACKING_H_

#include <cmath>
#include <string>
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

class LinearPositionWaypointTrajectory: public Trajectory {
 private:
    std::vector<double> waypointTimes;
    std::vector<Eigen::Vector3d> waypoints;

 public:
    LinearPositionWaypointTrajectory(
            std::vector<double> times,
            std::vector<Eigen::Vector3d> waypoints) {
        this->waypointTimes = times;
        this->waypoints = waypoints;

        assert(times.size() == waypoints.size());
    }

    LinearPositionWaypointTrajectory(const LinearPositionWaypointTrajectory& old) {
        this->waypointTimes = old.waypointTimes;
        this->waypoints = old.waypoints;
    }

    LinearPositionWaypointTrajectory(LinearPositionWaypointTrajectory&& other) = default;

    Simulator::RTN getTargetState(double t) override {
        size_t t1Index = std::upper_bound(
                this->waypointTimes.begin(),
                this->waypointTimes.end(),
                t) - this->waypointTimes.begin();

        // If t is greater than all of our waypoint times, return the last
        // waypoint
        if(t1Index == this->waypointTimes.size()) {
            Simulator::RTN targetState;
            targetState << this->waypoints.at(t1Index-1);
            targetState << 0.0, 0.0, 0.0;
            return targetState;
        }

        // If t is less than all of the times, return the first one.
        if(t1Index == 0) {
            Simulator::RTN targetState;
            targetState << this->waypoints.at(0);
            targetState << 0.0, 0.0, 0.0;
            return targetState;
        }

        // This is safe, since we checked that t1Index > 0
        size_t t0Index = t1Index - 1;

        // Now we can find the linear interpolant
        double t0 = this->waypointTimes.at(t0Index);
        double t1 = this->waypointTimes.at(t1Index);

        Eigen::Vector3d pos0 = this->waypoints.at(t0Index);
        Eigen::Vector3d pos1 = this->waypoints.at(t1Index);

        Eigen::Vector3d avgVelocity = (pos1 - pos0) / (t1 - t0);

        Eigen::Vector3d targetPos = pos0 + (t - t0)/(t1 - t0)*(pos1 - pos0);

        Simulator::RTN targetState;
        targetState << targetPos, avgVelocity;

        return targetState;
    }
};

class CSVTrajectory: public Trajectory {
 private:
    std::vector<double> waypointTimes;
    std::vector<Simulator::RTN> waypoints;

 public:
    CSVTrajectory(const std::string& fileName);

    CSVTrajectory(const CSVTrajectory& old) {
        this->waypointTimes = old.waypointTimes;
        this->waypoints = old.waypoints;
    }

    CSVTrajectory(CSVTrajectory&& other) = default;

    Simulator::RTN getTargetState(double t) override;
};


}

#endif
