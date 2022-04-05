#include <cmath>

#include "OrbitalState.h"
#include "Constants.h"
#include <Eigen/Dense>

namespace Simulator {

COE pvToCoe(const PV& pv) {
    // Position and velocity vectors and scalars
    Eigen::Vector3d r_vec = pv.head(3);
    double r = r_vec.norm();
    Eigen::Vector3d v_vec = pv.tail(3);
    double v = v_vec.norm();

    // Radial velocity
    double v_r = r_vec.dot(v_vec) / r;

    // Basic vectors
    Eigen::Vector3d h_vec = r_vec.cross(v_vec);
    double h = h_vec.norm();
    Eigen::Vector3d z_vec {0, 0, 1};
    Eigen::Vector3d n_vec = z_vec.cross(h_vec);
    double n = n_vec.norm();
    Eigen::Vector3d e_vec = (v*v/MU_EARTH - 1/r)*r_vec - (r_vec.dot(v_vec)/MU_EARTH)*v_vec;
    double e = e_vec.norm();

    // Orbital elements
    double i = std::acos(h_vec(2)/h);

    double Omega = std::acos(n_vec(0)/n);
    if(n_vec(1) < 0) {
        Omega = 2*PI - Omega;
    }

    double omega = std::acos(n_vec.dot(e_vec)/(n*e));
    if(e_vec(2) < 0) {
        omega = 2*PI - omega;
    }

    double f = std::acos(e_vec.dot(r_vec)/(e*r));
    if(v_r < 0) {
        f = 2*PI - f;
    }

    double a = h*h / (MU_EARTH*(1 - e*e));

    return {e, a, i, omega, Omega, f};
}

PV pvFromCoe(const COE& coe) {
    double e = coe(0);
    double a = coe(1);
    double i = coe(2);
    double omega = coe(3);
    double Omega = coe(4);
    double f = coe(5);

    double r = a*(1-e*e)/(1 + e * std::cos(f));

    Eigen::Vector3d r_pqw_vec {
        r*std::cos(f),
        r*std::sin(f),
        0.0
    };

    Eigen::Vector3d v_pqw_vec = {
        -1*std::sqrt(MU_EARTH/(a*(1 - e*e))) * std::sin(f),
        std::sqrt(MU_EARTH/(a*(1-e*e))) * (e + std::cos(f)),
        0.0
    };

    Eigen::Matrix3d T1;
    T1 << std::cos(omega),  -1*std::sin(omega), 0.0,
           std::sin(omega), std::cos(omega),    0.0,
           0.0,             0.0,                1.0;

    Eigen::Matrix3d T2;
    T2 << 1.0, 0.0,         0.0,
          0.0, std::cos(i), -1*std::sin(i),
          0.0, std::sin(i), std::cos(i);

    Eigen::Matrix3d T3;
    T3 << std::cos(Omega), -1*std::sin(Omega), 0.0,
          std::sin(Omega), std::cos(Omega),    0.0,
          0.0,             0.0,                1.0;

    Eigen::Vector3d r_eci_vec = T3*T2*T1*r_pqw_vec;
    Eigen::Vector3d v_eci_vec = T3*T2*T1*v_pqw_vec;

    PV pv;
    pv << r_eci_vec, v_eci_vec;

    return pv;
}

} // namespace Simulator
