#include "SimulationRuns.h"
#include "Simulator.h"
#include "OrbitalState.h"
#include "InfiniteLQR.h"
#include "InfiniteLQRLinearTracking.h"
#include "TrajectoryGeneration.h"

void below200InfiniteLQR(const std::string& fileName) {
    Simulator::COE targetCOE;
    targetCOE << 0.0, 8000e3, 0.0, 0.0, 0.0, 0.0;
    Simulator::COE chaserCOE;
    chaserCOE << 0.0, 7999.8e3, 0.0, 0.0, 0.0, 0.0;

    Simulator::PV target0 = Simulator::pvFromCoe(targetCOE);
    Simulator::PV chaser0 = Simulator::pvFromCoe(chaserCOE);
    auto target =  std::make_shared<Simulator::Vehicle>(1000.0, target0);
    auto chaser = std::make_shared<Controllers::InfiniteLQRVehicle>(
            100.0,         // Mass
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
    auto target =  std::make_shared<Simulator::Vehicle>(1000.0, target0);
    auto chaser = std::make_shared<Controllers::InfiniteLQRVehicle>(
            100.0,         // Mass
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
    auto target =  std::make_shared<Simulator::Vehicle>(1000.0, target0);
    auto chaser = std::make_shared<Controllers::InfiniteLQRVehicle>(
            100.0,         // Mass
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

void boxInfiniteLQRLinearTracking(const std::string& fileName) {
    Simulator::COE targetCOE;
    targetCOE << 0.0, 8000e3, 0.0, 0.0, 0.0, 0.0;
    Simulator::PV target0 = Simulator::pvFromCoe(targetCOE);

    // Make the set of waypoints
    std::vector<double> waypointTimes {
        0.0,
        1000.0,
        3000.0,
        5000.0,
        7000.0,
        8000.0,
        9000.0
    };
    Simulator::Vector6d waypoint0;
    Simulator::Vector6d waypoint1;
    Simulator::Vector6d waypoint2;
    Simulator::Vector6d waypoint3;
    Simulator::Vector6d waypoint4;
    Simulator::Vector6d waypoint5;
    Simulator::Vector6d waypoint6;
    waypoint0 << 0, 200, 0, 0, 0, 0;
    waypoint1 << 0, 200, 200, 0, 0, 0;
    waypoint2 << 0, -200, 200, 0, 0, 0;
    waypoint3 << 0, -200, -200, 0, 0, 0;
    waypoint4 << 0, 200, -200, 0, 0, 0;
    waypoint5 << 0, 200, 0, 0, 0, 0;
    waypoint6 << 0, 0, 0, 0, 0, 0;
    std::vector<Simulator::Vector6d> waypoints {
        waypoint0,
        waypoint1,
        waypoint2,
        waypoint3,
        waypoint4,
        waypoint5,
        waypoint6
    };

    auto trackedTrajectory = std::make_shared<Controllers::LinearWaypointTrajectory>(waypointTimes, waypoints);

    Simulator::RTN chaserRTN0;
    chaserRTN0 << 0, 200, 0, 0, 0, 0;
    Simulator::PV chaser0 = Simulator::pvFromRtn(chaserRTN0, target0);

    auto target =  std::make_shared<Simulator::Vehicle>(1000.0, target0);
    auto chaser = std::make_shared<Controllers::InfiniteLQRLinearTrackingVehicle>(
            100.0,         // Mass
            chaser0,       // Initial state
            targetCOE[1],
            trackedTrajectory);
    Simulator::Simulator sim(
            target,
            chaser,
            10.0,
            1.0);
    sim.setTrackedTrajectory(trackedTrajectory);

    sim.simulate(3500, true);
    sim.record.write(fileName);
}
