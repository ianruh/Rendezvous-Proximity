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
        double mass,
        Simulator::PV state,
        double targetSMA,
        std::shared_ptr<Trajectory> targetTrajectory,
        size_t numSteps,
        double timeStep): Simulator::Vehicle(mass, state) {
    // Target SMA needed to construct the linearized A matrix
    this->targetSMA = targetSMA;
    this->targetTrajectory = targetTrajectory;
    //double n = std::sqrt(MU_EARTH/std::pow(this->targetSMA, 3));
    double maxPos = 600.0;
    double maxAccel = 100.0;

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
    cppmpc::OrderedSet variableOrdering;
    for(size_t i = 0; i < numSteps; i++) {
        variableOrdering.append(x[i]);
        variableOrdering.append(y[i]);
        variableOrdering.append(z[i]);
        variableOrdering.append(vx[i]);
        variableOrdering.append(vy[i]);
        variableOrdering.append(vz[i]);
        variableOrdering.append(dx[i]);
        variableOrdering.append(dy[i]);
        variableOrdering.append(dz[i]);
    }

    //===== Parameters =====
    std::vector<SymEngine::Expression> initialState = cppmpc::toExpressions(cppmpc::variableVector("x_0", 6));

    std::vector<SymEngine::Expression> previousX = cppmpc::toExpressions(cppmpc::variableVector("x_prev", numSteps));
    std::vector<SymEngine::Expression> previousY = cppmpc::toExpressions(cppmpc::variableVector("y_prev", numSteps));
    std::vector<SymEngine::Expression> previousZ = cppmpc::toExpressions(cppmpc::variableVector("z_prev", numSteps));
    std::vector<SymEngine::Expression> previousVx = cppmpc::toExpressions(cppmpc::variableVector("vx_prev", numSteps));
    std::vector<SymEngine::Expression> previousVy = cppmpc::toExpressions(cppmpc::variableVector("vy_prev", numSteps));
    std::vector<SymEngine::Expression> previousVz = cppmpc::toExpressions(cppmpc::variableVector("vz_prev", numSteps));

    //====== Parameter Ordering ======
    cppmpc::OrderedSet parameterOrdering;
    for(size_t i = 0; i < 6; i++) {
        parameterOrdering.append(initialState[i]);
    }
    for(size_t i = 0; i < numSteps; i++) {
        parameterOrdering.append(previousX[i]);
        parameterOrdering.append(previousY[i]);
        parameterOrdering.append(previousZ[i]);
        parameterOrdering.append(previousVx[i]);
        parameterOrdering.append(previousVy[i]);
        parameterOrdering.append(previousVz[i]);
    }

    //===== Constraints =====
    
    // Initial conditions
    objective.equalityConstraints.appendConstraint(x[0], initialState[0]);
    objective.equalityConstraints.appendConstraint(y[0], initialState[1]);
    objective.equalityConstraints.appendConstraint(z[0], initialState[2]);
    objective.equalityConstraints.appendConstraint(vx[0], initialState[3]);
    objective.equalityConstraints.appendConstraint(vy[0], initialState[4]);
    objective.equalityConstraints.appendConstraint(vz[0], initialState[5]);

    // Set the maximum position and maximum force.
    for(size_t i = 0; i < numSteps; i++) {
        objective.inequalityConstraints.appendLessThan(x[i], maxPos);
        objective.inequalityConstraints.appendGreaterThan(x[i], -1*maxPos);
        objective.inequalityConstraints.appendLessThan(y[i], maxPos);
        objective.inequalityConstraints.appendGreaterThan(y[i], -1*maxPos);
        objective.inequalityConstraints.appendLessThan(z[i], maxPos);
        objective.inequalityConstraints.appendGreaterThan(z[i], -1*maxPos);

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
        
        //// Velocity Constraint
        //SymEngine::Expression k = 4.0/3.0;
        //SymEngine::Expression linear_acceleration_1 = gravity*taylorExpand(sin(angle[t-1])*cos(angle[t-1])/(pow(cos(angle[t-1]), integer(2)) - k*(cartMass/poleMass + 1.0)), angle[t-1], previousAngle[t], 1);
        //SymEngine::Expression linear_acceleration_2 = k*force[t-1]/(poleMass*pow(cos(previousAngle[t]), integer(2)) - k*(cartMass + poleMass));
        //SymEngine::Expression linear_acceleration_3 = k*poleLength*sin(previousAngle[t])/(pow(cos(previousAngle[t]), integer(2)) - k*(cartMass/poleMass + 1)) * taylorExpand(pow(angularVelocity[t-1], integer(2)), angularVelocity[t-1], previousAngularVelocity[t], 1);
        //
        //objective.equalityConstraints.appendConstraint(velocity[t], velocity[t-1] + dt*linear_acceleration_1 - dt*linear_acceleration_2 - dt*linear_acceleration_3);
        //
        //// Angle Constraint
        //objective.equalityConstraints.appendConstraint(angle[t], angle[t-1] + 0.5*dt*angularVelocity[t-1] + 0.5*dt*angularVelocity[t]);
        //
        //// Angular Velocity Constraint
        //SymEngine::Expression angular_acceleration_1 = gravity*(cartMass + poleMass) * taylorExpand(sin(angle[t-1])/(k*(cartMass+poleMass)*poleLength - poleMass*poleLength*pow(cos(angle[t-1]), integer(2))), angle[t-1], previousAngle[t], 1);
        //SymEngine::Expression angular_acceleration_2 = force[t-1]/(k*(cartMass + poleMass)*poleLength - poleMass*poleLength*cos(previousAngle[t]));
        //SymEngine::Expression angular_acceleration_3 = sin(previousAngle[t])/(k*(cartMass/poleMass + 1) - cos(previousAngle[t])) * taylorExpand(pow(angularVelocity[t-1], integer(2)), angularVelocity[t-1], previousAngularVelocity[t], 1);
        //
        //objective.equalityConstraints.appendConstraint(angularVelocity[t], angularVelocity[t-1] + dt*angular_acceleration_1 - dt*angular_acceleration_2 - dt*angular_acceleration_3);
        
    }
}

Eigen::Vector3d MPCNonLinearTrackingVehicle::getControl(
        [[maybe_unused]] double t,
        [[maybe_unused]] const Simulator::Vehicle& target) {
    // Construct the control
    Simulator::Vector6d state = this->getRtn(target.getPv());
    Simulator::Vector6d targetState = this->targetTrajectory->getTargetState(t);
    Simulator::Vector6d error = state - targetState;
    
    Eigen::Vector3d control = -1 * this->K * error;

    // This control is acceleration, so f=ma
    return control*this->mass;
}

} // namespace Controllers
