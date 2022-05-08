#include "MPCNonLinearTracking.h"
#include "Simulator.h"
#include "TrajectoryGeneration.h"
#include "Constants.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <optional>
#include <memory>
#include "RiccatiSolvers.h"

#include <cppmpc/SymbolicObjective.h>
#include <cppmpc/FastMPC.h>
#include <cppmpc/SymEngineUtilities.h>
#include <cppmpc/OrderedSet.h>
#include <symengine/basic.h>
#include <symengine/expression.h>
#include <symengine/functions.h>
#include <symengine/pow.h>

using cppmpc::sin;
using cppmpc::cos;
using cppmpc::tan;
using cppmpc::pow;
using cppmpc::taylorExpand;
using SymEngine::integer;

namespace Controllers {

MPCNonLinearTrackingVehicle::MPCNonLinearTrackingVehicle(
        Simulator::PV state,
        double targetSMA,
        size_t numSteps,
        double maxAccel,
        double timeStep,
        double finalStateWeight,
        double controlWeight): Simulator::Vehicle(state) {
    // Target SMA needed to construct the linearized A matrix
    this->targetSMA = targetSMA;
    //double n = std::sqrt(MU_EARTH/std::pow(this->targetSMA, 3));
    //double maxPos = 2000.0;

    // Just a constant
    auto n = SymEngine::Expression(SymEngine::real_double(std::sqrt(MU_EARTH/std::pow(this->targetSMA, 3))));

    //===== State Variables =====
    // These are numSteps length vectors with each element representing points in time
    std::vector<SymEngine::Expression> x = cppmpc::toExpressions(cppmpc::variableVector("x", numSteps));
    std::vector<SymEngine::Expression> y = cppmpc::toExpressions(cppmpc::variableVector("y", numSteps));
    std::vector<SymEngine::Expression> z = cppmpc::toExpressions(cppmpc::variableVector("z", numSteps));
    std::vector<SymEngine::Expression> vx = cppmpc::toExpressions(cppmpc::variableVector("vx", numSteps));
    std::vector<SymEngine::Expression> vy = cppmpc::toExpressions(cppmpc::variableVector("vy", numSteps));
    std::vector<SymEngine::Expression> vz = cppmpc::toExpressions(cppmpc::variableVector("vz", numSteps));

    //===== Control Variables ======
    std::vector<SymEngine::Expression> dx = cppmpc::toExpressions(cppmpc::variableVector("dx", numSteps));
    std::vector<SymEngine::Expression> dy = cppmpc::toExpressions(cppmpc::variableVector("dy", numSteps));
    std::vector<SymEngine::Expression> dz = cppmpc::toExpressions(cppmpc::variableVector("dz", numSteps));

    //===== Variable Ordering =====
    for(size_t i = 0; i < numSteps; i++) {
        this->variableOrdering.append(x[i]);
        this->variableOrdering.append(y[i]);
        this->variableOrdering.append(z[i]);
        this->variableOrdering.append(vx[i]);
        this->variableOrdering.append(vy[i]);
        this->variableOrdering.append(vz[i]);
        this->variableOrdering.append(dx[i]);
        this->variableOrdering.append(dy[i]);
        this->variableOrdering.append(dz[i]);
    }

    //===== Parameters =====
    std::vector<SymEngine::Expression> initialState = cppmpc::toExpressions(cppmpc::parameterVector("x_0", 6));

    //std::vector<SymEngine::Expression> previousX = cppmpc::toExpressions(cppmpc::variableVector("x_prev", numSteps));
    //std::vector<SymEngine::Expression> previousY = cppmpc::toExpressions(cppmpc::variableVector("y_prev", numSteps));
    //std::vector<SymEngine::Expression> previousZ = cppmpc::toExpressions(cppmpc::variableVector("z_prev", numSteps));
    //std::vector<SymEngine::Expression> previousVx = cppmpc::toExpressions(cppmpc::variableVector("vx_prev", numSteps));
    //std::vector<SymEngine::Expression> previousVy = cppmpc::toExpressions(cppmpc::variableVector("vy_prev", numSteps));
    //std::vector<SymEngine::Expression> previousVz = cppmpc::toExpressions(cppmpc::variableVector("vz_prev", numSteps));

    //====== Parameter Ordering ======
    for(size_t i = 0; i < 6; i++) {
        parameterOrdering.append(initialState[i]);
    }
    //for(size_t i = 0; i < numSteps; i++) {
    //    parameterOrdering.append(previousX[i]);
    //    parameterOrdering.append(previousY[i]);
    //    parameterOrdering.append(previousZ[i]);
    //    parameterOrdering.append(previousVx[i]);
    //    parameterOrdering.append(previousVy[i]);
    //    parameterOrdering.append(previousVz[i]);
    //}

    //===== Constraints =====
    
    // Initial conditions
    objective.equalityConstraints.appendConstraint(x[0], initialState[0]);
    objective.equalityConstraints.appendConstraint(y[0], initialState[1]);
    objective.equalityConstraints.appendConstraint(z[0], initialState[2]);
    objective.equalityConstraints.appendConstraint(vx[0], initialState[3]);
    objective.equalityConstraints.appendConstraint(vy[0], initialState[4]);
    objective.equalityConstraints.appendConstraint(vz[0], initialState[5]);

    //objective.equalityConstraints.appendConstraint(x[numSteps-1], 0.0);
    //objective.equalityConstraints.appendConstraint(y[numSteps-1], 0.0);
    //objective.equalityConstraints.appendConstraint(z[numSteps-1], 0.0);
    //objective.equalityConstraints.appendConstraint(vx[numSteps-1], 0.0);
    //objective.equalityConstraints.appendConstraint(vy[numSteps-1], 0.0);
    //objective.equalityConstraints.appendConstraint(vz[numSteps-1], 0.0);

    // Set the maximum position and maximum force.
    for(size_t i = 0; i < numSteps; i++) {
        //objective.inequalityConstraints.appendLessThan(x[i], maxPos);
        //objective.inequalityConstraints.appendGreaterThan(x[i], -1*maxPos);
        //objective.inequalityConstraints.appendLessThan(y[i], maxPos);
        //objective.inequalityConstraints.appendGreaterThan(y[i], -1*maxPos);
        //objective.inequalityConstraints.appendLessThan(z[i], maxPos);
        //objective.inequalityConstraints.appendGreaterThan(z[i], -1*maxPos);

        objective.inequalityConstraints.appendLessThan(dx[i], maxAccel);
        objective.inequalityConstraints.appendGreaterThan(dx[i], -1*maxAccel);
        objective.inequalityConstraints.appendLessThan(dy[i], maxAccel);
        objective.inequalityConstraints.appendGreaterThan(dy[i], -1*maxAccel);
        objective.inequalityConstraints.appendLessThan(dz[i], maxAccel);
        objective.inequalityConstraints.appendGreaterThan(dz[i], -1*maxAccel);
    }

    // Dynamics constraints
    const double dt = timeStep;
    for(size_t t = 1; t < numSteps; t++) {
        // Position constraint
        objective.equalityConstraints.appendConstraint(x[t], x[t-1] + 0.5*dt*vx[t-1] + 0.5*dt*vx[t]);
        objective.equalityConstraints.appendConstraint(y[t], y[t-1] + 0.5*dt*vy[t-1] + 0.5*dt*vy[t]);
        objective.equalityConstraints.appendConstraint(z[t], z[t-1] + 0.5*dt*vz[t-1] + 0.5*dt*vz[t]);

        // Velocity Constraint
        auto xdd_previous = 3*n*n*x[t-1] + 2*n*vy[t-1] + dx[t-1];
        auto xdd = 3*n*n*x[t] + 2*n*vy[t] + dx[t];
        objective.equalityConstraints.appendConstraint(vx[t], vx[t-1] + 0.5*dt*xdd_previous + 0.5*dt*xdd);

        auto ydd_previous = -2*n*vx[t-1] + dy[t-1];
        auto ydd = -2*n*vx[t] + dy[t];
        objective.equalityConstraints.appendConstraint(vy[t], vy[t-1] + 0.5*dt*ydd_previous + 0.5*dt*ydd);

        auto zdd_previous = -1*n*n*z[t-1] + dz[t-1];
        auto zdd = -1*n*n*z[t] + dz[t];
        objective.equalityConstraints.appendConstraint(vz[t], vz[t-1] + 0.5*dt*zdd_previous + 0.5*dt*zdd);
    }

    // Construct the objective
    //auto obj = finalStateWeight*(x[numSteps-1]*x[numSteps-1] +
    //    y[numSteps-1]*y[numSteps-1] +
    //    z[numSteps-1]*z[numSteps-1] +
    //    vx[numSteps-1]*vx[numSteps-1] +
    //    vy[numSteps-1]*vy[numSteps-1] +
    //    vz[numSteps-1]*vz[numSteps-1]);
    auto obj = SymEngine::Expression(SymEngine::real_double(0.0));
    for(size_t i = 0; i < numSteps; i++) {
        obj = obj + finalStateWeight*(x[i]*x[i] +
            y[i]*y[i] +
            z[i]*z[i] +
            vx[i]*vx[i] +
            vy[i]*vy[i] +
            vz[i]*vz[i]);
        obj = obj + controlWeight*(dx[i]*dx[i] + dy[i]*dy[i] + dz[i]*dz[i]);
    }
    objective.setObjective(obj);

    Simulator::Vector6d initialParamValues = Simulator::Vector6d::Zero();
    objective.setParameters(initialParamValues);

    objective.finalize(variableOrdering, parameterOrdering);

    this->solver = std::make_unique<cppmpc::FastMPC::Solver>(objective);
    this->solver->hyperParameters.newtonStepsStageMaximum = 10;
}

Eigen::Vector3d MPCNonLinearTrackingVehicle::getControl(
        [[maybe_unused]] double t,
        [[maybe_unused]] const Simulator::Vehicle& target) {
    // Construct the control
    Simulator::Vector6d state = this->getRtn(target.getPv());

    // Parameters are just the initial state
    this->objective.setParameters(state);

    auto [minimum, primal, dual] = this->solver->minimize(
            this->previousPrimal, this->previousDual);

    this->previousPrimal = primal;
    this->previousDual = dual;

    Eigen::Vector3d control;
    control << primal(6), primal(7), primal(8);

    return control;
}

} // namespace Controllers
