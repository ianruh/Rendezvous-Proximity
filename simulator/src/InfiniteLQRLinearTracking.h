#ifndef SRC_INFINITE_LQR_LINEAR_TRACKING_H_
#define SRC_INFINITE_LQR_LINEAR_TRACKING_H_
#include "OrbitalState.h"
#include "Simulator.h"
#include "TrajectoryGeneration.h"
#include <memory>
#include <Eigen/Dense>

namespace Controllers {

class InfiniteLQRLinearTrackingVehicle: public Simulator::Vehicle {
 private:
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 3> B;
    Eigen::Matrix<double, 6, 6> Pf;
    Eigen::Matrix<double, 6, 6> Q;
    Eigen::Matrix<double, 3, 3> R;
    double targetSMA;

    Eigen::Matrix<double, 3, 6> K;

    std::shared_ptr<Trajectory> targetTrajectory;

 public:

    InfiniteLQRLinearTrackingVehicle(
            Simulator::PV state,
            double targetSMA,
            std::shared_ptr<Trajectory> targetTrajectory);

    Eigen::Vector3d getControl(
            [[maybe_unused]] double t,
            [[maybe_unused]] const Simulator::Vehicle& target) override;

};

} // namespace Controllers

#endif // SRC_INFINITE_LQR_LINEAR_TRACKING_H_
