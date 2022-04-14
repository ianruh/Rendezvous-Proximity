#ifndef SRC_RICCATI_SOLVERS_H_
#define SRC_RICCATI_SOLVERS_H_
#include <Eigen/Dense>
#include <optional>

namespace Controllers {


std::optional<Eigen::MatrixXd> solveRiccatiIteration(
    const Eigen::MatrixXd &A,
    const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R,
    double dt = 0.001,
    double tolerance = 1.E-5,
    size_t iter_max = 100000);

std::optional<Eigen::MatrixXd> solveRiccatiEigen(
        const Eigen::MatrixXd &A,
        const Eigen::MatrixXd &B,
        const Eigen::MatrixXd &Q,
        const Eigen::MatrixXd &R);

}

#endif // SRC_RICCATI_SOLVERS_H_
