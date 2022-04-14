#ifndef SRC_FINITELQR_H_
#define SRC_FINITELQR_H_
#include "OrbitalState.h"
#include "Simulator.h"
#include <Eigen/Dense>

namespace Controllers {

class InfiniteLQRVehicle: public Simulator::Vehicle {
 private:
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 3> B;
    Eigen::Matrix<double, 6, 6> Pf;
    Eigen::Matrix<double, 6, 6> Q;
    Eigen::Matrix<double, 3, 3> R;
    double targetSMA;

    Eigen::Matrix<double, 3, 6> K;

 public:

    InfiniteLQRVehicle(
            double mass,
            Simulator::PV state,
            double targetSMA,
            double alpha = 1.0/(100.0*100.0),
            double beta = 1.0/(0.1*0.1));

    Eigen::Vector3d getControl(
            [[maybe_unused]] double t,
            [[maybe_unused]] const Simulator::Vehicle& target) override;

};

} // namespace Controllers

#endif // SRC_FINITELQR_H_
