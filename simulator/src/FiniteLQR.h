#ifndef SRC_FINITELQR_H_
#define SRC_FINITELQR_H_
#include "OrbitalState.h"
#include "Simulator.h"
#include <Eigen/Dense>

namespace Controllers {

class FiniteLQRVehicle: public Simulator::Vehicle {
 private:
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 3> B;
    Eigen::Matrix<double, 6, 6> Pf;
    Eigen::Matrix<double, 6, 6> Q;
    Eigen::Matrix<double, 3, 3> R;
    double targetSMA;
    double timeHorizon;

    std::vector<Eigen::Matrix<double, 6,6>> Pt;
    std::vector<double> times;

 public:

    FiniteLQRVehicle(
            Simulator::PV state,
            double targetSMA,
            double timeHorizon);

    Eigen::Vector3d getControl(
            [[maybe_unused]] double t,
            [[maybe_unused]] const Simulator::Vehicle& target) override;

    typedef Eigen::Matrix<double, 6, 6> state_type;
    void riccatiIteration(
            double t0,
            double tf);

    Eigen::Matrix<double, 6, 6> interpolateP(double t) const;

};

} // namespace Controllers

#endif // SRC_FINITELQR_H_
