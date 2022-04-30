#ifndef SRC_MPC_NON_LINEAR_TRACKING_H_
#define SRC_MPC_NON_LINEAR_TRACKING_H_
#include "OrbitalState.h"
#include "Simulator.h"
#include "TrajectoryGeneration.h"
#include <memory>
#include <optional>
#include <Eigen/Dense>
#include <cppmpc/FastMPC.h>
#include <cppmpc/SymbolicObjective.h>
#include <cppmpc/OrderedSet.h>

namespace Controllers {

class MPCNonLinearTrackingVehicle: public Simulator::Vehicle {
 private:
    double targetSMA;

    // The objective function
    cppmpc::FastMPC::SymbolicObjective objective;

    // Solver
    std::unique_ptr<cppmpc::FastMPC::Solver> solver;

    // Orderings
    cppmpc::OrderedSet variableOrdering;
    cppmpc::OrderedSet parameterOrdering;

    // Cache primals
    mutable std::optional<Eigen::VectorXd> previousPrimal;
    mutable std::optional<Eigen::VectorXd> previousDual;

 public:

    MPCNonLinearTrackingVehicle(
            Simulator::PV state,
            double targetSMA,
            size_t numSteps = 30,
            double maxAccel = 0.01,
            double timeStep = 20.0,
            double finalStateWeight = 100.0,
            double controlWeight = 0.01);

    Eigen::Vector3d getControl(
            [[maybe_unused]] double t,
            [[maybe_unused]] const Simulator::Vehicle& target) override;

};

} // namespace Controllers

#endif // SRC_MPC_NON_LINEAR_TRACKING_H_

