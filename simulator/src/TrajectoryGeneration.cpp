#include "TrajectoryGeneration.h"

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "Simulator.h"
#include "OrbitalState.h"
#include "external/rapidcsv.h"

namespace Controllers {

CSVTrajectory::CSVTrajectory(const std::string& fileName) {
    rapidcsv::Document doc(fileName);

    this->waypointTimes = doc.GetColumn<double>("time");
    std::vector<double> x = doc.GetColumn<double>("x");
    std::vector<double> y = doc.GetColumn<double>("y");
    std::vector<double> z = doc.GetColumn<double>("z");
    std::vector<double> vx = doc.GetColumn<double>("vx");
    std::vector<double> vy = doc.GetColumn<double>("vy");
    std::vector<double> vz = doc.GetColumn<double>("vz");

    this->waypoints = std::vector<Simulator::RTN>();

    for(size_t i = 0; i < this->waypointTimes.size(); i++) {
        Simulator::RTN state;
        state <<
            x[i],
            y[i],
            z[i],
            vx[i],
            vy[i],
            vz[i];
        this->waypoints.push_back(state);
    }
}


Simulator::RTN CSVTrajectory::getTargetState(double t) {
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

    Simulator::RTN state0 = this->waypoints.at(t0Index);
    Simulator::RTN state1 = this->waypoints.at(t1Index);

    Simulator::RTN interpState = state0 + (t - t0)/(t1 - t0)*(state1 - state0);

    return interpState;
}

} // namespace Controllers
