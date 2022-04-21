#include "SimulationRuns.h"
#include "Simulator.h"
#include "OrbitalState.h"
#include "InfiniteLQR.h"
#include "InfiniteLQRLinearTracking.h"
#include "InfiniteLQRNonLinearTracking.h"
#include "TrajectoryGeneration.h"
#include "Constants.h"
#include <Eigen/Dense>
#include <cmath>

void boxGeoInfiniteLQRLinearTracking(const std::string& fileName) {
    boxInfiniteLQRLinearTracking(fileName, 42164e3, 8000);
}
void box30000InfiniteLQRLinearTracking(const std::string& fileName) {
    boxInfiniteLQRLinearTracking(fileName, 30000e3, 8000);
}
void box20000InfiniteLQRLinearTracking(const std::string& fileName) {
    boxInfiniteLQRLinearTracking(fileName, 20001e3, 8000);
}
void box10000InfiniteLQRLinearTracking(const std::string& fileName) {
    boxInfiniteLQRLinearTracking(fileName, 10000e3, 8000);
}

void below200InfiniteLQR(const std::string& fileName) {
    Simulator::COE targetCOE;
    targetCOE << 0.0, 8000e3, 0.0, 0.0, 0.0, 0.0;
    Simulator::COE chaserCOE;
    chaserCOE << 0.0, 7999.8e3, 0.0, 0.0, 0.0, 0.0;

    Simulator::PV target0 = Simulator::pvFromCoe(targetCOE);
    Simulator::PV chaser0 = Simulator::pvFromCoe(chaserCOE);
    auto target =  std::make_shared<Simulator::Vehicle>(target0);
    auto chaser = std::make_shared<Controllers::InfiniteLQRVehicle>(
            chaser0,       // Initial state
            targetCOE[1]); // Target SMA
    Simulator::Simulator sim(
            target,
            chaser,
            10.0,
            1.0);

    sim.simulate(5000, true);
    sim.record.write(fileName);
}

void above200InfiniteLQR(const std::string& fileName) {
    Simulator::COE targetCOE;
    targetCOE << 0.0, 8000e3, 0.0, 0.0, 0.0, 0.0;
    Simulator::COE chaserCOE;
    chaserCOE << 0.0, 8000.2e3, 0.0, 0.0, 0.0, 0.0;

    Simulator::PV target0 = Simulator::pvFromCoe(targetCOE);
    Simulator::PV chaser0 = Simulator::pvFromCoe(chaserCOE);
    auto target =  std::make_shared<Simulator::Vehicle>(target0);
    auto chaser = std::make_shared<Controllers::InfiniteLQRVehicle>(
            chaser0,       // Initial state
            targetCOE[1]); // Target SMA
    Simulator::Simulator sim(
            target,
            chaser,
            10.0,
            1.0);

    sim.simulate(500, true);
    sim.record.write(fileName);
}

void above20InfiniteLQR(const std::string& fileName) {
    Simulator::COE targetCOE;
    targetCOE << 0.0, 8000e3, 0.0, 0.0, 0.0, 0.0;
    Simulator::COE chaserCOE;
    chaserCOE << 0.0, 8000.02e3, 0.0, 0.0, 0.0, 0.0;

    Simulator::PV target0 = Simulator::pvFromCoe(targetCOE);
    Simulator::PV chaser0 = Simulator::pvFromCoe(chaserCOE);
    auto target =  std::make_shared<Simulator::Vehicle>(target0);
    auto chaser = std::make_shared<Controllers::InfiniteLQRVehicle>(
            chaser0,       // Initial state
            targetCOE[1]); // Target SMA
    Simulator::Simulator sim(
            target,
            chaser,
            10.0,
            1.0);

    sim.simulate(5000, true);
    sim.record.write(fileName);
}

void leading2000InfiniteLQRLinearTracking(const std::string& fileName) {
    Simulator::COE targetCOE;
    targetCOE << 0.0, 8000e3, 0.0, 0.0, 0.0, 0.0;
    //Simulator::COE chaserCOE;
    //chaserCOE << 0.0, 8000e3, 0.0, 0.0, 0.0, 0.001/4.0;
    Simulator::RTN chaserRTN;
    chaserRTN << 0.0, 2000.0, 0.0, 0.0, 0.0, 0.0;

    Simulator::PV target0 = Simulator::pvFromCoe(targetCOE);
    //Simulator::PV chaser0 = Simulator::pvFromCoe(chaserCOE);
    Simulator::PV chaser0 = Simulator::pvFromRtn(chaserRTN, target0);
    auto target =  std::make_shared<Simulator::Vehicle>(target0);

    auto trackedTrajectory = std::make_shared<Controllers::CSVTrajectory>("../trajectories/2000mLeadingRendezvous.csv");

    auto chaser = std::make_shared<Controllers::InfiniteLQRLinearTrackingVehicle>(
            chaser0,       // Initial state
            targetCOE[1], // Target SMA
            trackedTrajectory);

    Simulator::Simulator sim(
            target,
            chaser,
            10.0,
            1.0);

    sim.setTrackedTrajectory(trackedTrajectory);
    sim.simulate(3000, true);
    sim.record.write(fileName);
}

void leading2000InfiniteLQRNonLinearTracking(const std::string& fileName) {
    Simulator::COE targetCOE;
    targetCOE << 0.0, 8000e3, 0.0, 0.0, 0.0, 0.0;
    //Simulator::COE chaserCOE;
    //chaserCOE << 0.0, 8000e3, 0.0, 0.0, 0.0, 0.001/4.0;
    Simulator::RTN chaserRTN;
    chaserRTN << 0.0, 2000.0, 0.0, 0.0, 0.0, 0.0;

    Simulator::PV target0 = Simulator::pvFromCoe(targetCOE);
    //Simulator::PV chaser0 = Simulator::pvFromCoe(chaserCOE);
    Simulator::PV chaser0 = Simulator::pvFromRtn(chaserRTN, target0);
    auto target =  std::make_shared<Simulator::Vehicle>(target0);

    auto trackedTrajectory = std::make_shared<Controllers::CSVTrajectory>("../trajectories/2000mLeadingRendezvous.csv");

    auto chaser = std::make_shared<Controllers::InfiniteLQRNonLinearTrackingVehicle>(
            chaser0,       // Initial state
            targetCOE(1), // Target SMA
            trackedTrajectory);

    Simulator::Simulator sim(
            target,
            chaser,
            10.0,
            1.0);

    sim.setTrackedTrajectory(trackedTrajectory);
    sim.simulate(7121, true);
    sim.record.write(fileName);
}

void boxInfiniteLQRLinearTracking(
        const std::string& fileName,
        double sma,
        double boxDuration) {
    Simulator::COE targetCOE;
    targetCOE << 0.0, sma, 0.0, 0.0, 0.0, 0.0;
    Simulator::PV target0 = Simulator::pvFromCoe(targetCOE);

    double dt = boxDuration/8;

    // Make the set of waypoints
    std::vector<double> waypointTimes {
        0.0,
        dt,
        3*dt,
        5*dt,
        7*dt,
        boxDuration
    };
    Eigen::Vector3d waypoint0;
    Eigen::Vector3d waypoint1;
    Eigen::Vector3d waypoint2;
    Eigen::Vector3d waypoint3;
    Eigen::Vector3d waypoint4;
    Eigen::Vector3d waypoint5;
    waypoint0 << 0, 200, 0;         // T = 0
    waypoint1 << 200, 200, 0;
    waypoint2 << 200, -200, 0;
    waypoint3 << -200, -200, 0;
    waypoint4 << -200, 200, 0;
    waypoint5 << 0, 200, 0;         // T = T
    std::vector<Eigen::Vector3d> waypoints {
        waypoint0,
        waypoint1,
        waypoint2,
        waypoint3,
        waypoint4,
        waypoint5
    };

    auto trackedTrajectory = std::make_shared<Controllers::LinearPositionWaypointTrajectory>(waypointTimes, waypoints);

    Simulator::RTN chaserRTN0;
    chaserRTN0 << 0, 200, 0, 0, 0, 0;
    Simulator::PV chaser0 = Simulator::pvFromRtn(chaserRTN0, target0);

    auto target = std::make_shared<Simulator::Vehicle>(target0);
    auto chaser = std::make_shared<Controllers::InfiniteLQRLinearTrackingVehicle>(
            chaser0,       // Initial state
            targetCOE[1],
            trackedTrajectory);
    Simulator::Simulator sim(
            target,
            chaser,
            10.0,
            1.0);
    sim.setTrackedTrajectory(trackedTrajectory);

    sim.simulate(boxDuration, true);
    sim.record.write(fileName);
}


void boxInfiniteLQRNonLinearTracking(
        const std::string& fileName,
        double sma,
        double boxDuration) {
    Simulator::COE targetCOE;
    targetCOE << 0.0, sma, 0.0, 0.0, 0.0, 0.0;
    Simulator::PV target0 = Simulator::pvFromCoe(targetCOE);

    double dt = boxDuration/8;

    // Make the set of waypoints
    std::vector<double> waypointTimes {
        0.0,
        dt,
        3*dt,
        5*dt,
        7*dt,
        boxDuration
    };
    Eigen::Vector3d waypoint0;
    Eigen::Vector3d waypoint1;
    Eigen::Vector3d waypoint2;
    Eigen::Vector3d waypoint3;
    Eigen::Vector3d waypoint4;
    Eigen::Vector3d waypoint5;
    waypoint0 << 0, 200, 0;         // T = 0
    waypoint1 << 200, 200, 0;
    waypoint2 << 200, -200, 0;
    waypoint3 << -200, -200, 0;
    waypoint4 << -200, 200, 0;
    waypoint5 << 0, 200, 0;         // T = T
    std::vector<Eigen::Vector3d> waypoints {
        waypoint0,
        waypoint1,
        waypoint2,
        waypoint3,
        waypoint4,
        waypoint5
    };

    auto trackedTrajectory = std::make_shared<Controllers::LinearPositionWaypointTrajectory>(waypointTimes, waypoints);

    Simulator::RTN chaserRTN0;
    chaserRTN0 << 0, 200, 0, 0, 0, 0;
    Simulator::PV chaser0 = Simulator::pvFromRtn(chaserRTN0, target0);

    auto target =  std::make_shared<Simulator::Vehicle>(target0);
    auto chaser = std::make_shared<Controllers::InfiniteLQRNonLinearTrackingVehicle>(
            chaser0,       // Initial state
            targetCOE[1],
            trackedTrajectory);
    Simulator::Simulator sim(
            target,
            chaser,
            10.0,
            1.0);
    sim.setTrackedTrajectory(trackedTrajectory);

    sim.simulate(boxDuration, true);
    sim.record.write(fileName);
}
