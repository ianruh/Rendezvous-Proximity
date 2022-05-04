#include "InfiniteLQRNonLinearTracking.h"
#include "Simulator.h"
#include "TrajectoryGeneration.h"
#include "Constants.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <optional>
#include <memory>
#include "RiccatiSolvers.h"
#include <symengine/basic.h>
#include <symengine/expression.h>
#include <symengine/matrix.h>
#include <symengine/symbol.h>
#include <symengine/integer.h>
#include <symengine/pow.h>
#include <symengine/mul.h>
#include <symengine/real_double.h>
#include <symengine/subs.h>
#include <cppmpc/SymEngineUtilities.h>
#include <cppmpc/OrderedSet.h>
#include "CodeGen.h"

using SymEngine::Basic;
using SymEngine::Expression;
using SymEngine::RCP;
using SymEngine::Symbol;

namespace Controllers {

SymEngine::DenseMatrix matrixSubs(const SymEngine::DenseMatrix& original,
        const SymEngine::map_basic_basic& map) {
    SymEngine::DenseMatrix newMat(original.nrows(), original.ncols());

    for(size_t row = 0; row < original.nrows(); row++) {
        for(size_t col = 0; col < original.ncols(); col++) {
            newMat.set(row, col, SymEngine::subs(original.get(row, col), map));
        }
    }

    return newMat;
}

InfiniteLQRNonLinearTrackingVehicle::InfiniteLQRNonLinearTrackingVehicle(
        Simulator::PV state,
        double targetSMA,
        std::shared_ptr<Trajectory> targetTrajectory,
        Eigen::Matrix<double, 6, 6> Q,
        Eigen::Matrix<double, 3, 3> R): Simulator::Vehicle(state) {
    // Target SMA needed to construct the linearized A matrix
    this->targetSMA = targetSMA;
    this->targetTrajectory = targetTrajectory;

    // Some constants
    auto n = Expression(SymEngine::real_double(std::sqrt(MU_EARTH/std::pow(this->targetSMA, 3))));
    auto mu = Expression(SymEngine::real_double(MU_EARTH));
    auto rc = Expression(SymEngine::real_double(targetSMA));

    //============ Symbolic Variables and Parameters ===========
    // Create the symbolic state variables and ordering
    auto x = Expression(cppmpc::variable("x"));
    auto y = Expression(cppmpc::variable("y"));
    auto z = Expression(cppmpc::variable("z"));
    auto vx = Expression(cppmpc::variable("vx"));
    auto vy = Expression(cppmpc::variable("vy"));
    auto vz = Expression(cppmpc::variable("vz"));
    // Create the symbolic error state variables
    auto ex = Expression(cppmpc::variable("ex"));
    auto ey = Expression(cppmpc::variable("ey"));
    auto ez = Expression(cppmpc::variable("ez"));
    auto evx = Expression(cppmpc::variable("evx"));
    auto evy = Expression(cppmpc::variable("evy"));
    auto evz = Expression(cppmpc::variable("evz"));
    
    // Create the symbolic control variables
    auto dr = Expression(cppmpc::variable("dr"));
    auto dt = Expression(cppmpc::variable("dt"));
    auto dn = Expression(cppmpc::variable("dn"));
    // Create the symbolic error control variables
    auto edr = Expression(cppmpc::variable("edr"));
    auto edt = Expression(cppmpc::variable("edt"));
    auto edn = Expression(cppmpc::variable("edn"));

    // Symbolic parameters for the desired state
    auto xd = Expression(cppmpc::parameter("xd"));
    auto yd = Expression(cppmpc::parameter("yd"));
    auto zd = Expression(cppmpc::parameter("zd"));
    auto vxd = Expression(cppmpc::parameter("vxd"));
    auto vyd = Expression(cppmpc::parameter("vyd"));
    auto vzd = Expression(cppmpc::parameter("vzd"));
    // Symbolic parameters for the desired control
    auto drd = Expression(cppmpc::parameter("drd"));
    auto dtd = Expression(cppmpc::parameter("dtd"));
    auto dnd = Expression(cppmpc::parameter("dnd"));

    cppmpc::OrderedSet stateParameterOrdering;
    stateParameterOrdering.append(xd);
    stateParameterOrdering.append(yd);
    stateParameterOrdering.append(zd);
    stateParameterOrdering.append(vxd);
    stateParameterOrdering.append(vyd);
    stateParameterOrdering.append(vzd);
    cppmpc::OrderedSet controlParameterOrdering;
    controlParameterOrdering.append(drd);
    controlParameterOrdering.append(dtd);
    controlParameterOrdering.append(dnd);

    cppmpc::OrderedSet stateErrorOrdering;
    stateErrorOrdering.append(ex);
    stateErrorOrdering.append(ey);
    stateErrorOrdering.append(ez);
    stateErrorOrdering.append(evx);
    stateErrorOrdering.append(evy);
    stateErrorOrdering.append(evz);
    cppmpc::OrderedSet controlErrorOrdering;
    controlErrorOrdering.append(edr);
    controlErrorOrdering.append(edt);
    controlErrorOrdering.append(edn);

    //========== Symbolic Dynamics ========
    // Some rused quantities
    auto threeHalves = Expression(SymEngine::div(SymEngine::integer(3), SymEngine::integer(2)));
    auto denom = SymEngine::pow((rc+x)*(rc+x) + y*y + z*z, threeHalves);

    // State space form of system dynamics
    auto f1 = vx;
    auto f2 = vy;
    auto f3 = vz;
    auto f4 = (-1 * mu * (rc + x)) / denom + mu/(rc*rc) + 2*n*vy + n*n*x + dr;
    auto f5 = (-1 * mu * y) / denom + 2*n*vx + n*n*y + dt;
    auto f6 = (-1* mu * z) / denom + dn;
    SymEngine::DenseMatrix f(6, 1);
    f.set(0, 0, f1.get_basic());
    f.set(1, 0, f2.get_basic());
    f.set(2, 0, f3.get_basic());
    f.set(3, 0, f4.get_basic());
    f.set(4, 0, f5.get_basic());
    f.set(5, 0, f6.get_basic());

    //========== Error Dynamics ===========
    SymEngine::map_basic_basic xToxdeMap;
    xToxdeMap[x.get_basic()] = (xd + ex).get_basic();
    xToxdeMap[y.get_basic()] = (yd + ey).get_basic();
    xToxdeMap[z.get_basic()] = (zd + ez).get_basic();
    xToxdeMap[vx.get_basic()] = (vxd + evx).get_basic();
    xToxdeMap[vy.get_basic()] = (vyd + evy).get_basic();
    xToxdeMap[vz.get_basic()] = (vzd + evz).get_basic();
    xToxdeMap[dr.get_basic()] = (drd + edr).get_basic();
    xToxdeMap[dt.get_basic()] = (dtd + edt).get_basic();
    xToxdeMap[dn.get_basic()] = (dnd + edn).get_basic();

    SymEngine::map_basic_basic xToxdMap;
    xToxdMap[x.get_basic()] = xd.get_basic();
    xToxdMap[y.get_basic()] = yd.get_basic();
    xToxdMap[z.get_basic()] = zd.get_basic();
    xToxdMap[vx.get_basic()] = vxd.get_basic();
    xToxdMap[vy.get_basic()] = vyd.get_basic();
    xToxdMap[vz.get_basic()] = vzd.get_basic();
    xToxdMap[dr.get_basic()] = drd.get_basic();
    xToxdMap[dt.get_basic()] = dtd.get_basic();
    xToxdMap[dn.get_basic()] = dnd.get_basic();

    // fde = f(xd + e, ud + v)
    SymEngine::DenseMatrix fde = matrixSubs(f, xToxdeMap);
    
    // fd = f(xd, ud)
    SymEngine::DenseMatrix fd = matrixSubs(f, xToxdMap);

    // F = fde - fd
    SymEngine::DenseMatrix negativefd(6, 1);
    fd.mul_scalar(SymEngine::integer(-1), negativefd);
    SymEngine::DenseMatrix F(6, 1);
    fde.add_matrix(fd, F);

    //========= Linearized Error Dynamics =========
    SymEngine::DenseMatrix dFdestate = cppmpc::jacobian(F, stateErrorOrdering);
    SymEngine::DenseMatrix dFdecontrol = cppmpc::jacobian(F, controlErrorOrdering);

    // Set the errors to 0
    SymEngine::map_basic_basic errorTo0Map;
    errorTo0Map[ex.get_basic()] = SymEngine::integer(0);
    errorTo0Map[ey.get_basic()] = SymEngine::integer(0);
    errorTo0Map[ez.get_basic()] = SymEngine::integer(0);
    errorTo0Map[evx.get_basic()] = SymEngine::integer(0);
    errorTo0Map[evy.get_basic()] = SymEngine::integer(0);
    errorTo0Map[evz.get_basic()] = SymEngine::integer(0);
    errorTo0Map[edr.get_basic()] = SymEngine::integer(0);
    errorTo0Map[edt.get_basic()] = SymEngine::integer(0);
    errorTo0Map[edn.get_basic()] = SymEngine::integer(0);
    SymEngine::DenseMatrix AtSymbolic = matrixSubs(dFdestate, errorTo0Map);
    SymEngine::DenseMatrix BtSymbolic = matrixSubs(dFdecontrol, errorTo0Map);

    //========== Code Generation =========
    this->aMatrixFunction = getParameterizedFunction(
            AtSymbolic, stateParameterOrdering);
    this->bMatrixFunction = getParameterizedFunction(
            BtSymbolic, controlParameterOrdering);

    //============ Weight Matrices =========
    this->Q = Q;
    this->R = R;
}

Eigen::Vector3d InfiniteLQRNonLinearTrackingVehicle::getControl(
        [[maybe_unused]] double t,
        [[maybe_unused]] const Simulator::Vehicle& target) {
    // Construct the control
    Simulator::Vector6d state = this->getRtn(target.getPv());
    Simulator::Vector6d targetState = this->targetTrajectory->getTargetState(t);
    Simulator::Vector6d error = state - targetState;

    // We assume the target control is 0
    Eigen::Vector3d targetControl;
    targetControl << 0.0, 0.0, 0.0;

    // Get the parameterized matrices
    Eigen::Matrix<double, 6, 6> A(6,6);
    (*this->aMatrixFunction)(targetState.data(), A.data());
    Eigen::Matrix<double, 6, 3> B(6,3);
    (*this->bMatrixFunction)(targetControl.data(), B.data());

    // Solve the riccati equation
    auto P = solveRiccatiEigen(A, B, Q, R);
    if(!P) {
        throw std::runtime_error("Riccati Iteration failed to converge.");
    }

    Eigen::Matrix<double, 3, 6> K = this->R.inverse() * B.transpose() * P.value();
    
    Eigen::Vector3d control = -1 * K * error;

    // This control is acceleration
    return control;
}

} // namespace Controllers
