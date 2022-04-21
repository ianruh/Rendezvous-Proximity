#ifndef SRC_INFINITE_LQR_LINEAR_TRACKING_H_
#define SRC_INFINITE_LQR_LINEAR_TRACKING_H_
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
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 3> B;
    Eigen::Matrix<double, 6, 6> Pf;
    Eigen::Matrix<double, 6, 6> Q;
    Eigen::Matrix<double, 3, 3> R;
    double targetSMA;

    Eigen::Matrix<double, 3, 6> K;

    std::shared_ptr<Trajectory> targetTrajectory;

    // The objective function
    cppmpc::FastMPC::SymbolicObjective objective;

 public:

    MPCNonLinearTrackingVehicle(
            Simulator::PV state,
            double targetSMA,
            std::shared_ptr<Trajectory> targetTrajectory,
            size_t numSteps = 20,
            double timeStep = 100.0);

    Eigen::Vector3d getControl(
            [[maybe_unused]] double t,
            [[maybe_unused]] const Simulator::Vehicle& target) override;

};

} // namespace Controllers

#endif // SRC_INFINITE_LQR_LINEAR_TRACKING_H_
