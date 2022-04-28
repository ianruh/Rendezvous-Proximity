#ifndef SRC_INFINITE_LQR_H_
#define SRC_INFINITE_LQR_H_
#include "OrbitalState.h"
#include "Simulator.h"
#include <Eigen/Dense>

namespace Controllers {

class InfiniteLQRVehicle: public Simulator::Vehicle {
 private:
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 3> B;
    Eigen::Matrix<double, 6, 6> Pf;
    double targetSMA;

    Eigen::Matrix<double, 3, 6> K;

 public:

    InfiniteLQRVehicle(
            Simulator::PV state,
            double targetSMA,
            Eigen::Matrix<double, 6, 6> Q,
            Eigen::Matrix<double, 3, 3> R);

    Eigen::Vector3d getControl(
            [[maybe_unused]] double t,
            [[maybe_unused]] const Simulator::Vehicle& target) override;

};

} // namespace Controllers

#endif // SRC_INFINITE_LQR_H_
