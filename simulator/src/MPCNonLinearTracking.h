#ifndef SRC_MPC_NON_LINEAR_TRACKING_H_
#define SRC_MPC_NON_LINEAR_TRACKING_H_
#include "OrbitalState.h"
#include "Simulator.h"
#include "TrajectoryGeneration.h"
#include <memory>
#include <Eigen/Dense>
#include <cppmpc/FastMPC.h>
#include <cppmpc/SymbolicObjective.h>

namespace Controllers {

class MPCNonLinearTrackingVehicle: public Simulator::Vehicle {
 private:
    double targetSMA;

    // The objective function
    cppmpc::FastMPC::SymbolicObjective objective;

 public:

    MPCNonLinearTrackingVehicle(
            Simulator::PV state,
            double targetSMA,
            size_t numSteps = 20,
            double maxAccel = 0.01,
            double timeStep = 100.0);

    Eigen::Vector3d getControl(
            [[maybe_unused]] double t,
            [[maybe_unused]] const Simulator::Vehicle& target) override;

};

} // namespace Controllers

#endif // SRC_MPC_NON_LINEAR_TRACKING_H_

