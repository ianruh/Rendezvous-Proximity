#include "RiccatiSolvers.h"
#include <Eigen/Dense>
#include <optional>

// Adapted from https://github.com/TakaHoribe/Riccati_Solver/blob/master/riccati_solver.cpp

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

std::optional<Eigen::MatrixXd> solveRiccatiEigen(
        const Eigen::MatrixXd &A,
        const Eigen::MatrixXd &B,
        const Eigen::MatrixXd &Q,
        const Eigen::MatrixXd &R) {
    Eigen::MatrixXd P;
    
    const size_t dim_x = A.rows();
    //const size_t dim_u = B.cols();
    
    // set Hamilton matrix
    Eigen::MatrixXd Ham = Eigen::MatrixXd::Zero(2 * dim_x, 2 * dim_x);
    Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();
    
    // calc eigenvalues and eigenvectors
    Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);
    
    // extract stable eigenvectors into 'eigvec'
    Eigen::MatrixXcd eigvec = Eigen::MatrixXcd::Zero(2 * dim_x, dim_x);
    size_t j = 0;
    for (size_t i = 0; i < 2 * dim_x; ++i) {
        if (Eigs.eigenvalues()[i].real() < 0.) {
            eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * dim_x, 1);
            ++j;
        }
    }
    
    // calc P with stable eigen vector matrix
    Eigen::MatrixXcd Vs_1, Vs_2;
    Vs_1 = eigvec.block(0, 0, dim_x, dim_x);
    Vs_2 = eigvec.block(dim_x, 0, dim_x, dim_x);
    P = (Vs_2 * Vs_1.inverse()).real();
    
    return P;
}

} // namespace Controllers
