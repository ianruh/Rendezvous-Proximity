#include "RiccatiSolvers.h"
#include <Eigen/Dense>
#include <optional>

namespace Controllers {

std::optional<Eigen::MatrixXd> solveRiccatiIteration(
        const Eigen::MatrixXd &A,
        const Eigen::MatrixXd &B,
        const Eigen::MatrixXd &Q,
        const Eigen::MatrixXd &R,
        double dt,
        double tolerance,
        size_t iter_max) {
    Eigen::MatrixXd P = Q; // initialize

    Eigen::MatrixXd P_next;

    Eigen::MatrixXd AT = A.transpose();
    Eigen::MatrixXd BT = B.transpose();
    Eigen::MatrixXd Rinv = R.inverse();

    double diff;
    for (uint i = 0; i < iter_max; ++i) {
        P_next = P + (P * A + AT * P - P * B * Rinv * BT * P + Q) * dt;
        diff = fabs((P_next - P).maxCoeff());
        P = P_next;
        if (diff < tolerance) {
            return P;
        }
    }
    return std::optional<Eigen::MatrixXd>(); // over iteration limit
}

} // namespace Controllers
