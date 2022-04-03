#ifndef SRC_ORBITAL_STATE_H_
#define SRC_ORBITAL_STATE_H_
#include <Eigen/Dense>

namespace Simulator {

/** Orbital elemnt conventions
 *
 * PV: [x, y, z, vx, vy, vz]
 *
 * COE: [e, a, i, w, raan, f]
 */

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Vector6d PV;
typedef Vector6d COE;

COE pvToCoe(const PV& pv);

PV pvFromCoe(const COE& coe);

} // namespace Simulator

#endif // SRC_ORBITAL_STATE_H_
